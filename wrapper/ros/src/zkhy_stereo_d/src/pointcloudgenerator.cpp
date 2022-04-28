#include "pointcloudgenerator.h"

#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "smarter_eye_sdk/satpext.h"
#include "smarter_eye_sdk/disparityconvertor.h"

PointCloudGenerator::PointCloudGenerator()
    : width_(0),
      height_(0),
      bit_num_(0),
      running_(false),
      frame_ready_(false),
      disparity_raw_data_(nullptr)
{

}

PointCloudGenerator::~PointCloudGenerator()
{
    stop();
}

void PointCloudGenerator::init(int width, int height,  int dataSize, int bitNum)
{
    width_ = width;
    height_ = height;
    bit_num_ = bitNum;

    disparity_raw_data_.reset((RawImageFrame*)new char[sizeof(RawImageFrame) + dataSize]);

    start();
}

void PointCloudGenerator::push(const RawImageFrame *rawFrame)
{
    if (!prepared()) return;

    if (!running_ || frame_ready_) return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (frame_ready_) return;

        memcpy(disparity_raw_data_.get(), rawFrame, sizeof(RawImageFrame) + rawFrame->dataSize);
        frame_ready_ = true;
    }

    condition_.notify_one();
}

void PointCloudGenerator::start()
{
    if (!prepared()) return;

    if (running_) return;

    {
        std::lock_guard<std::mutex> _(mutex_);
        if (running_) return;
        running_ = true;
    }
    thread_ = std::thread(&PointCloudGenerator::run, this);
}

void PointCloudGenerator::stop()
{
    if (!running_) return;

    {
        std::lock_guard<std::mutex> _(mutex_);
        if (!running_) return;
        running_ = false;
        frame_ready_ = true;
    }
    condition_.notify_one();
    if (thread_.joinable()) {
        thread_.join();
    }
}

void PointCloudGenerator::run()
{
    while (running_) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]{ return frame_ready_; });
            if (!running_) break;
        }

        sensor_msgs::PointCloud2::Ptr pc(new sensor_msgs::PointCloud2);
        pc->header.stamp = ros::Time(disparity_raw_data_->time / 1000.0);
        pc->width = width_;
        pc->height = height_;

        sensor_msgs::PointCloud2Modifier modifier(*pc);
        modifier.setPointCloud2Fields(3,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32);

        sensor_msgs::PointCloud2Iterator<float> iter_x(*pc, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pc, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pc, "z");

        for (int posY = 0; posY < height_; posY++) {
            for (int posX = 0; posX < width_; posX++) {
                float x, y, z;
                DisparityConvertor::getPointXYZDistance(disparity_raw_data_->image, width_, height_, bit_num_,
                                                        calib_params_.Tx, calib_params_.focus, calib_params_.cx, calib_params_.cy,
                                                        posX, posY, x, y, z);

                if ((fabs(x) > 200000.0f)||(fabs(y) > 200000.0f)||(fabs(z) > 200000.0f)) {
                    x = 0.0f;
                    y = 0.0f;
                    z = 0.0f;
                }

                *iter_x = x / 1000.f;
                *iter_y = y / 1000.f;
                *iter_z = z / 1000.f;

                ++iter_x; ++iter_y; ++iter_z;
            }
        }

        publish_callback_(pc);

        {
            std::lock_guard<std::mutex> _(mutex_);
            frame_ready_ = false;
        }
    }
}
