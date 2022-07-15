#include "framemonitor.h"

#include <iostream>
#include <opencv2/imgproc.hpp>

#include "satpext.h"
#include "frameid.h"
#include "frameformat.h"
#include "yuv2rgb.h"
#include "disparityconvertor.h"
#include "frameext.h"

FrameMonitor::FrameMonitor()
    : mFrameReadyFlag(false),
      mDisparityFloatData(new float[1280 * 720]),
      mDisparityDistanceZ(new float[1280 * 720]),
      mRgbBuffer(new unsigned char[1280 * 720 * 3]),
      mPointCloudGenerator(new PointCloudGenerator)
{
    // support gray or rgb
    mLeftMat.create(720, 1280, CV_8UC3);
    mRightMat.create(720, 1280, CV_8UC3);
    mDisparityMat.create(720, 1280, CV_8UC3);
}

void FrameMonitor::handleRawFrame(const RawImageFrame *rawFrame)
{
    processFrame(rawFrame);
}

void FrameMonitor::processFrame(const RawImageFrame *rawFrame)
{
    int64_t timestamp = rawFrame->time;
    size_t size = rawFrame->index;
    switch (rawFrame->frameId) {
    case FrameId::Disparity:
    {
        // only for FrameFormat::Disparity16, bitNum = 5
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mDisparityMat);

        mFrameCallback(FrameId::Disparity, timestamp, mDisparityMat);

        if (!mPointCloudGenerator->prepared()) {
            mPointCloudGenerator->init(rawFrame->width, rawFrame->height, rawFrame->dataSize,
                    DisparityConvertor::getDisparityBitNum(rawFrame->format));
        }
        mPointCloudGenerator->push(rawFrame);

      //  std::cout << "update disparity mat" << std::endl;
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
        break;
    case FrameId::CalibLeftCamera:
    {
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mLeftMat);
        mFrameCallback(FrameId::CalibLeftCamera, timestamp, mLeftMat);

     //   std::cout << "update left mat" << std::endl;
        char *extended = (char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame);
        const FrameDataExtHead *header = reinterpret_cast<const FrameDataExtHead *>(extended);
        if (header->dataType == FrameDataExtHead::Compound)
        {
            header = reinterpret_cast<const FrameDataExtHead *>(header->data);
            size -= sizeof (FrameDataExtHead);
        }
       do{
            if(header->dataType == FrameDataExtHead::MotionData)
            {
                const MotionData *motionPtr = reinterpret_cast<const MotionData *>(header->data);
                int totalNum = (header->dataSize - sizeof(FrameDataExtHead)) / sizeof(MotionData);
                for(int index = 0; index < totalNum; index++)
                {
                    MotionData data(motionPtr[index].accelX,motionPtr[index].accelY,motionPtr[index].accelZ,
                                    motionPtr[index].gyroX,motionPtr[index].gyroY,motionPtr[index].gyroZ,motionPtr[index].timestamp);
                    mMotionDataCallback(&data);
                }
            }
            header = reinterpret_cast<const FrameDataExtHead*>(reinterpret_cast<const char*>(header) + header->dataSize);
        }while(reinterpret_cast<const char*>(header) < (extended + size));

        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
        break;
    case FrameId::CalibRightCamera:
    {
        std::lock_guard<std::mutex> lock(mMutex);
        loadFrameData2Mat(rawFrame, mRightMat);
        mFrameCallback(FrameId::CalibRightCamera, timestamp, mRightMat);

    //    std::cout << "update right mat" << std::endl;
        mFrameReadyFlag = true;
        mFrameReadyCond.notify_one();
    }
        break;
    }
}

void FrameMonitor::waitForFrames()
{
    std::unique_lock<std::mutex> lock(mMutex);
    mFrameReadyCond.wait(lock, [this]{
        return mFrameReadyFlag;
    });

    mFrameReadyFlag = false;
}

cv::Mat FrameMonitor::getFrameMat(int frameId)
{
    std::lock_guard<std::mutex> lock(mMutex);

    switch (frameId) {
    case FrameId::CalibRightCamera:
        return mRightMat.clone();
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
    {
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
//        cv::cvtColor(dstMat, dstMat, CV_RGB2BGR);
    }
        break;
    case FrameFormat::YUV422Plannar:
    {
        YuvToRGB::YCbYCrPlannar2Rgb(imageData, (char *)mRgbBuffer.get(), width, height);
        cv::Mat yuv422PlannarMat(height, width, CV_8UC3, mRgbBuffer.get());
        cv::resize(yuv422PlannarMat, dstMat, dstMat.size());
//        cv::cvtColor(dstMat, dstMat, CV_RGB2BGR);
    }
        break;
    }
}
