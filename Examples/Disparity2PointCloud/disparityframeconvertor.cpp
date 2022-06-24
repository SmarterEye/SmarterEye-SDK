
#include "disparityframeconvertor.h"
#include <QFile>
#include <QDebug>
#include <iostream>
#include "disparityconvertor.h"
#include <string>
#include <math.h>
#include "common.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


static const int kDisprityWidth4Bit5 = 1280;
static const int kDisprityHeight4Bit5 = 720;
//
static const int kDisparityCount = 81;
static const int kMaxCloudQueueSize = 10;

DisparityFrameConvertor::DisparityFrameConvertor():
    is_lookup_table_generated_(false),
    x_distance_buffer_(new float[1280*720]),
    y_distance_buffer_(new float[1280*720]),
    z_distance_buffer_(new float[1280*720])
{

}

DisparityFrameConvertor::~DisparityFrameConvertor()
{
    delete [] x_distance_buffer_;
    delete [] y_distance_buffer_;
    delete [] z_distance_buffer_;

    x_distance_buffer_ = nullptr;
    y_distance_buffer_ = nullptr;
    z_distance_buffer_ = nullptr;
}

void DisparityFrameConvertor::convertOneFrame(const QString &fileSrc, const QString &fileDest, int format)
{
    switch(format)
    {
    case FileFormat::dat:
    {
        QFile disparityFile(fileSrc);
        if (!disparityFile.open(QIODevice::ReadOnly)) return;
        QByteArray disparityData = disparityFile.readAll();
        qint64 fileSize = disparityFile.size();
        disparityFile.close();

        int width, height;
        int bitNum = getBitNumByFileSize(fileSize);
        if (bitNum == 5) {
            width = kDisprityWidth4Bit5;
            height = kDisprityHeight4Bit5;
        } else {
            // not support yet
            qDebug() << "bitnum:" << bitNum << "not support yet";
            return;
        }
        handleDisparityByLookupTable((uchar *)disparityData.data(),width,height,bitNum);
    }
        break;

     case FileFormat::png:
    {
        cv::Mat dispMat = cv::imread(fileSrc.toStdString(),CV_LOAD_IMAGE_UNCHANGED);
        uchar * disparityData = dispMat.data;
        const uchar * dataStart = dispMat.datastart;
        const uchar *dataEnd = dispMat.dataend;
        qint64 fileSize = dataEnd - dataStart;
        int width, height;
        int bitNum = getBitNumByFileSize(fileSize);
        if (bitNum == 5) {
            width = kDisprityWidth4Bit5;
            height = kDisprityHeight4Bit5;
        } else {
            // not support yet
            qDebug() << "bitnum:" << bitNum << "not support yet";
            return;
        }

        handleDisparityByLookupTable(disparityData,width,height,bitNum);
    }
        break;
    default:
        return;
    }
    cloud_path_queue_.push(fileDest);
    cloud_ready_cond_.notify_one();
}

int DisparityFrameConvertor::getBitNumByFileSize(qint64 fileSize) const
{
    int bitNum = 0;
    switch (fileSize) {
    case FileSizeByBitNum::Bit5:
        bitNum = 5;
        break;
    default:
        break;
    }

    return bitNum;
}

void DisparityFrameConvertor::setStereoCalibParams(StereoCalibrationParameters &params)
{
    calib_params_ = params;
}

void DisparityFrameConvertor::handleDisparityByLookupTable(unsigned char *image, int width, int height, int bitNum)
{
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
        //filter
        float x, y, z;
        for (int i = 0; i < width * height; i++) {
            x = x_distance_buffer_[i];
            y = y_distance_buffer_[i];
            z = z_distance_buffer_[i];
        if ((fabs(x) > 200000.0f)||(fabs(y) > 200000.0f)||(fabs(z) > 200000.0f))
        {
             continue;
        }
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->push_back(point);
            }
        cloud_queue_.push(cloud);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DisparityFrameConvertor::getCloud()
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

QString DisparityFrameConvertor::GetCloudPath()
{
    std::unique_lock<std::mutex> lock(mutex_);
    cloud_ready_cond_.wait(lock, [this]{
        return !cloud_path_queue_.empty();
    });

    QString path = cloud_path_queue_.front();
    cloud_path_queue_.pop();
    return path;
}

int DisparityFrameConvertor::GetQueueSize()
{
    return cloud_queue_.size();
}

