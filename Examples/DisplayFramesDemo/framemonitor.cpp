#include "framemonitor.h"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "satpext.h"
#include "frameid.h"
#include "frameformat.h"
#include "disparityconvertor.h"
#include "yuv2rgb.h"

FrameMonitor::FrameMonitor()
    : mFrameReadyFlag(false),
      mDisparityFloatData(new float[1280 * 720]),
      mDisparityDistanceZ(new float[1280 * 720]),
      mRgbBuffer(new unsigned char[1280 * 720 * 3])
{
    // support gray or rgb
    mLeftMat.create(720/2, 1280/2, CV_8UC3);
    mDisparityMat.create(720/2, 1280/2, CV_8UC3);
}

void FrameMonitor::handleRawFrame(const RawImageFrame *rawFrame)
{
    processFrame(rawFrame);
}

void FrameMonitor::processFrame(const RawImageFrame *rawFrame)
{
    switch (rawFrame->frameId) {
    case FrameId::Disparity:
    {
        // only for FrameFormat::Disparity16, bitNum = 5
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mDisparityMat);

//        std::cout << "update disparity mat" << std::endl;
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
        break;
    case FrameId::CalibLeftCamera: //保留左图，去除右图输出能力
    {
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mLeftMat);

//        std::cout << "update left mat" << std::endl;
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
        break;
    }
}

void FrameMonitor::waitForFrames()
{
    std::unique_lock<std::mutex> lock(mMutex);
    mFrameReadyCond.wait_for(lock, std::chrono::milliseconds(240), [this]{
        return mFrameReadyFlag;
    });

    mFrameReadyFlag = false;
}

cv::Mat FrameMonitor::getFrameMat(int frameId)
{
    std::lock_guard<std::mutex> lock(mMutex);

    switch (frameId) {
    case FrameId::Disparity:
        return mDisparityMat.clone();
    default:
        return mLeftMat.clone();
    }
}

void FrameMonitor::loadFrameData2Mat(const RawImageFrame *frameData, cv::Mat &dstMat)
{
    int width = frameData->width;
    int height = frameData->height;
    const unsigned char *imageData = frameData->image;

    switch (frameData->format) {
    case FrameFormat::Disparity16:
    case FrameFormat::DisparityDens16:
    {
//        DisparityConvertor::convertDisparity2FloatFormat(imageData, width, height, 5, mDisparityFloatData.get());
//        DisparityConvertor::convertDisparity2RGB(mDisparityFloatData.get(), width, height, 0, 45, mRgbBuffer.get());
//        cv::Mat dispMat(height, width, CV_8UC3, mRgbBuffer.get());
//        cv::resize(dispMat, dstMat, dstMat.size());

        cv::Mat dispMat(height, width, CV_16U, (void*)imageData);
        cv::resize(dispMat, dstMat, dstMat.size());
        cv::normalize(dstMat, dstMat, 0, 255, cv::NORM_MINMAX, CV_8U);
    }
        break;
    case FrameFormat::Gray:
    {
        cv::Mat grayMat(height, width, CV_8UC1, (void*)imageData);
        cv::resize(grayMat, dstMat, dstMat.size());
    }
        break;
    case FrameFormat::YUV422:
    {
        YuvToRGB::YCbYCr2Rgb(imageData, (char *)mRgbBuffer.get(), width, height);
        cv::Mat yuv422Mat(height, width, CV_8UC3, mRgbBuffer.get());
        cv::resize(yuv422Mat, dstMat, dstMat.size());
        cv::cvtColor(dstMat, dstMat, CV_RGB2BGR);
    }
        break;
    case FrameFormat::YUV422Plannar:
    {
        YuvToRGB::YCbYCrPlannar2Rgb(imageData, (char *)mRgbBuffer.get(), width, height);
        cv::Mat yuv422PlannarMat(height, width, CV_8UC3, mRgbBuffer.get());
        cv::resize(yuv422PlannarMat, dstMat, dstMat.size());
        cv::cvtColor(dstMat, dstMat, CV_RGB2BGR);
    }
        break;
    }
}
