//
// Created by xtp on 2020/4/23.
//

#ifndef ZKHY_STEREO_D_POINTCLOUDGENERATOR_H
#define ZKHY_STEREO_D_POINTCLOUDGENERATOR_H

#include <thread>
#include <mutex>
#include <condition_variable>

#include <sensor_msgs/PointCloud2.h>
#include "smarter_eye_sdk/calibrationparams.h"

struct RawImageFrame;

class PointCloudGenerator
{
public:
    PointCloudGenerator();
    ~PointCloudGenerator();

    void init(int width, int height, int dataSize, int bitNum);

    bool prepared() const
    {
        return width_ > 0 && height_ > 0 && bit_num_ > 0 && disparity_raw_data_ != nullptr;
    }

    void setCalibParams(StereoCalibrationParameters &params) {calib_params_ = params;}

    void setCallback(std::function<void(sensor_msgs::PointCloud2::Ptr)> cb) {publish_callback_ = std::move(cb);}

    void push(const RawImageFrame *rawFrame);

    void start();

    void stop();

    // thread func
    void run();

private:
    int width_;
    int height_;
    int bit_num_;

    bool running_;
    bool frame_ready_;
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable condition_;

    std::function<void(sensor_msgs::PointCloud2::Ptr)> publish_callback_;
    std::shared_ptr<RawImageFrame> disparity_raw_data_;
    StereoCalibrationParameters calib_params_;
};


#endif //ZKHY_STEREO_D_POINTCLOUDGENERATOR_H
