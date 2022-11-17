#ifndef FRAMEMONITOR_H
#define FRAMEMONITOR_H

#include <thread>
#include <mutex>
#include <memory>
#include <functional>
#include <condition_variable>
#include <opencv2/core.hpp>
#include <utility>
#include <sensor_msgs/PointCloud2.h>

#include "camerahandler.h"
#include "motiondata.h"
#include "calibrationparams.h"
#include "pointcloudgenerator.h"


class FrameMonitor : public CameraHandler
{
public:
    FrameMonitor();
    virtual ~FrameMonitor() {}

    // image handler func in recv qt thread of SATP Protocol(based on tcp)
    void handleRawFrame(const RawImageFrame *rawFrame);

    // custom function for processing recieved frame data in handleRawFrame()
    void processFrame(const RawImageFrame *rawFrame);

    void waitForFrames();

    cv::Mat getFrameMat(int frameId);

    void setStereoCalibParams(StereoCalibrationParameters &params)
    {
        mCalibParams = params;
        mPointCloudGenerator->setCalibParams(params);
    }

    void setFrameCallback(std::function<void(int frameId, int64_t timestamp, const cv::Mat &imgMat)> cb)
    {
        mFrameCallback = std::move(cb);
    }

    void setMotionDataCallback(std::function<void(const MotionData *motionData)> cb)
    {
        mMotionDataCallback = std::move(cb);
    }

    void setPointCloudCallback(std::function<void(sensor_msgs::PointCloud2::Ptr)> cb)
    {
        mPointCloudGenerator->setCallback(std::move(cb));
    }

protected:
    void loadFrameData2Mat(const RawImageFrame *frameData, cv::Mat &dstMat);

private:
    std::mutex mMutex;
    std::condition_variable mFrameReadyCond;
    bool mFrameReadyFlag;

    cv::Mat mLeftMat;
    cv::Mat mRightMat;
    cv::Mat mDisparityMat;

    std::function<void(int frameId, int64_t timestamp, const cv::Mat &imgMat)> mFrameCallback;
    std::function<void(const MotionData *motionData)> mMotionDataCallback;

    std::unique_ptr<float[]> mDisparityFloatData;
    std::unique_ptr<float []> mDisparityDistanceZ;
    std::unique_ptr<unsigned char[]> mRgbBuffer;

    std::unique_ptr<PointCloudGenerator> mPointCloudGenerator;
    StereoCalibrationParameters mCalibParams;
};

#endif // FRAMEMONITOR_H
