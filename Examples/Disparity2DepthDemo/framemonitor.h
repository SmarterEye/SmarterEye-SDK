#ifndef FRAMEMONITOR_H
#define FRAMEMONITOR_H

#include <thread>
#include <mutex>
#include <memory>
#include <condition_variable>
#include <opencv2/core.hpp>

#include "stereocamera.h"
#include "camerahandler.h"
#include "calibrationparams.h"

class FrameMonitor : public CameraHandler
{
public:
    FrameMonitor();
    virtual ~FrameMonitor() {}

    // image handler func in recv thread of SATP Protocol(based on tcp)
    void handleRawFrame(const RawImageFrame *rawFrame);

    // custom function for processing recieved frame data in handleRawFrame()
    void processFrame(const RawImageFrame *rawFrame);

    // the draw function must be called in main thread loop!!!
    void waitForFrames();

    cv::Mat getFrameMat(int frameId,StereoCamera *camera);
    
    

protected:
    void loadFrameData2Mat(const RawImageFrame *frameData, cv::Mat &dstMat);
    cv::Mat display2DepthRgbMat(const cv::Mat &dispMat, const StereoCalibrationParameters &mparams);

private:
    std::mutex mMutex;
    std::condition_variable mFrameReadyCond;
    bool mFrameReadyFlag;

    cv::Mat mDisparityMat;

    std::unique_ptr<float[]> mDisparityFloatData;
    std::unique_ptr<float []> mDisparityDistanceZ;
    std::unique_ptr<unsigned char[]> mRgbBuffer;
};

#endif // FRAMEMONITOR_H
