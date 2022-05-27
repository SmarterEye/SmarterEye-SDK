#include <iostream>
#include <math.h>
#include <cstdlib>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"
#include "frameid.h"
#include "disparityconvertor.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

static const int kDisparityCount = 81;
static const int kMaxCloudQueueSize = 10;

MyCameraHandler::MyCameraHandler(const std::string &name):
    name_(name),
    is_lookup_table_generated_(false),
    is_calib_param_ready_(false),
    x_distance_buffer_(new float[1280*720]),
    y_distance_buffer_(new float[1280*720]),
    z_distance_buffer_(new float[1280*720])
{
}

MyCameraHandler::~MyCameraHandler()
{
    delete [] x_distance_buffer_;
    delete [] y_distance_buffer_;
    delete [] z_distance_buffer_;

    x_distance_buffer_ = nullptr;
    y_distance_buffer_ = nullptr;
    z_distance_buffer_ = nullptr;
}

void MyCameraHandler::setStereoCalibParams(StereoCalibrationParameters &params)
{
    calib_params_ = params;
    is_calib_param_ready_ = true;
    std::cout  << "calib params is ready!!!" << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MyCameraHandler::getCloud()
{    
    std::unique_lock<std::mutex> lock(mutex_);
    cloud_ready_cond_.wait(lock, [this]{
        return !cloud_queue_.empty();
    });

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cloud_queue_.front();
    cloud_queue_.pop();
    if (cloud_queue_.size() < kMaxCloudQueueSize) {
        cloud_queue_spare_cond_.notify_one();
    }

    return cloud;
}

void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
    processFrame(rawFrame->frameId, (char*)rawFrame + sizeof(RawImageFrame),
                 rawFrame->dataSize, rawFrame->width, rawFrame->height, rawFrame->format);
}

void MyCameraHandler::processFrame(int frameId, char *image, uint32_t dataSize, int width, int height, int frameFormat)
{
    (void)dataSize;

    switch (frameId) {
    case FrameId::Disparity:
    {
        int bitNum = DisparityConvertor::getDisparityBitNum(frameFormat);
        if (is_calib_param_ready_) {
            handleDisparityByLookupTable((unsigned char *)image, width, height, bitNum);
        } else {
            std::cout  << "calib params is not ready!!!" << std::endl;
        }
    }
        break;
    default:
        break;
    }
}

void MyCameraHandler::handleDisparityByLookupTable(unsigned char *image, int width, int height, int bitNum)
{
//    std::cout << "width: " << width << ", height: " << height << std::endl;

    // generate X Y Z lookup table, only once is OK
    static float *lookUpTableX = new float[kDisparityCount*(int)pow(2, bitNum)*width];
    static float *lookUpTableY = new float[kDisparityCount*(int)pow(2, bitNum)*height];
    static float *lookUpTableZ = new float[kDisparityCount*(int)pow(2, bitNum)];
    if (!is_lookup_table_generated_) {
        DisparityConvertor::generateLookUpTableX(width, bitNum, calib_params_.Tx, calib_params_.cx, lookUpTableX);
        DisparityConvertor::generateLookUpTableY(height, bitNum, calib_params_.Tx, calib_params_.cy, lookUpTableY);
        DisparityConvertor::generateLookUpTableZ(bitNum, calib_params_.Tx, calib_params_.focus, lookUpTableZ);
        is_lookup_table_generated_ = true;
    }

    // get X Y Z distance according to the lookup table
    DisparityConvertor::getWholeXDistanceByLookupTable(image, width, height, bitNum, lookUpTableX, x_distance_buffer_);
    DisparityConvertor::getWholeYDistanceByLookupTable(image, width, height, bitNum, lookUpTableY, y_distance_buffer_);
    DisparityConvertor::getWholeZDistanceByLookupTable(image, width, height, lookUpTableZ, z_distance_buffer_);

    // load point cloud data
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cloud_queue_spare_cond_.wait(lock, [this]{
            return cloud_queue_.size() < kMaxCloudQueueSize;
        });

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        float x, y, z;

        for (int i = 0; i < width * height; i++) {
            x = x_distance_buffer_[i];
            y = y_distance_buffer_[i];
            z = z_distance_buffer_[i];

            if ((fabs(x) > 200000.0f)||(fabs(y) > 200000.0f)||(fabs(z) > 200000.0f)){
                //std::cout << "PointXYZ is null" << std::endl;
                continue;
            }

            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud->push_back(point);
        }
		
        cloud_queue_.push(cloud);
        cloud_ready_cond_.notify_one();
    }
}
