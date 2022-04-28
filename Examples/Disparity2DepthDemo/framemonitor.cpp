#include "framemonitor.h"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "stereocamera.h"
#include "satpext.h"
#include "frameid.h"
#include "frameformat.h"
#include "disparityconvertor.h"
#include "yuv2rgb.h"
#include "calibrationparams.h"

FrameMonitor::FrameMonitor()
    : mFrameReadyFlag(false),
      mDisparityFloatData(new float[1280 * 720]),
      mDisparityDistanceZ(new float[1280 * 720]),
      mRgbBuffer(new unsigned char[1280 * 720 * 3])
{
    // support gray or rgb
    mDisparityMat.create(720, 1280, CV_16U);
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

       //std::cout << "update disparity mat" << std::endl;
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

cv::Mat FrameMonitor::getFrameMat(int frameId,StereoCamera *camera)
{
    std::lock_guard<std::mutex> lock(mMutex);

    switch (frameId) 
{
    case FrameId::DepthRGB:{                //将实际返回的视差图转换为深度图,以满足请求需要的深度图
    cv::Mat disparity=mDisparityMat.clone();

    StereoCalibrationParameters params;
    camera->requestStereoCameraParameters(params);//参数包括相机基线和焦距

    cv::Mat depthRGB = display2DepthRgbMat(disparity,params);//视差数据转深度彩图矩阵

    return depthRGB;
        }

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
//        DisparityConvertor::convertDisparity2FloatFormat(imageData, width, height, 5, mDisparityFloatData.get());
//        DisparityConvertor::convertDisparity2RGB(mDisparityFloatData.get(), width, height, 0, 45, mRgbBuffer.get());
//        cv::Mat dispMat(height, width, CV_8UC3, mRgbBuffer.get());
//        cv::resize(dispMat, dstMat, dstMat.size());

        cv::Mat dispMat(height, width, CV_16U, (void*)imageData);
        dstMat=dispMat.clone();
        //cv::resize(dispMat, dstMat, dstMat.size());
        //cv::normalize(dstMat, dstMat, 0, 255, cv::NORM_MINMAX, CV_8U);
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


cv::Mat FrameMonitor::display2DepthRgbMat(const cv::Mat &dispMat, const StereoCalibrationParameters &mparams)
{
    float baseline=mparams.Tx;  //   获得基线值
    float fx=mparams.focus;  //获得焦距值

    cv::Mat depth(720, 1280, CV_32FC1);
    cv::Mat depthGray(720,1280,CV_8U);
    cv::Mat mdepthRGB(720,1280,CV_8U);

    for (int row = 0; row < depth.rows; row++)
    {
        for (int col = 0; col < depth.cols; col++)
        {
            ushort disp= dispMat.ptr<ushort>(row)[col];
            float  fdisp = float(int((disp & 0x0FFF) >> 5)) + float((disp & 0x1F)) / 32.0;//计算浮点型的真实深度值公式

            if (abs(fdisp) >4.0){                                                       //4.0是改善深度图观感的参数，可修改
                depth.ptr<float>(row)[col] = fx * baseline / fdisp;
            }
        }
    }
    cv::normalize(depth, depthGray, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::applyColorMap(depthGray, mdepthRGB, cv::COLORMAP_JET);

    return mdepthRGB;

}