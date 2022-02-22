#ifndef STEREOCAMERA_H
#define STEREOCAMERA_H

#include "stereocameradef.h"
#include "sedevicestate.h"
#include <cstdint>
#include <functional>
#include "deviceinfo.h"

class StereoCameraImpl;
class CameraHandler;
class FrameHandler;
struct MonoCalibrationParameters;
struct StereoCalibrationParameters;
struct RotationMatrix;
struct DeviceInfo;

namespace SATP {
    class Protocol;
}

class STEREO_SHARED_EXPORT StereoCamera
{
public:
    StereoCamera(const StereoCamera&&){}
    virtual ~StereoCamera();
    static StereoCamera *connect(const char *addr);

    void invokeInLoopThread(std::function<void()> method);

    void disconnectFromServer();
    bool isConnected();
    void requestFrame(FrameHandler *frameHandler, uint32_t frameIds);
    SATP::Protocol *getProtocol();
    void reboot(bool halt = false);
    void switchStartupMode();
    int updateFirmware(const char* path);
    double getUpgradeProgress();
    void setFileReceiveDir(const char *dir);
    void enableTasks(uint32_t taskIds);
    bool requestStereoCameraParameters(StereoCalibrationParameters &params);
    bool requestMonoLeftCameraParameters(MonoCalibrationParameters &params);
    bool requestMonoRightCameraParameters(MonoCalibrationParameters &params);
    bool requestRotationMatrix(RotationMatrix &params);
    void enableMaxSendFrameInterval();
    bool setFrameRate(float rate);
    bool getFrameRate(float &rate);
    bool getAmbientLight(int &lightness);
    bool getSmudgeStatus(int &status);
    void setImuAccelRange(int value);
    void setImuRotationRange(int value);
    void setImuReadFrequence(int value);
    int getImuAccelRange();
    int getImuRotationRange();
    int getImuReadFrequence();
    void enableMotionData(bool enable);
    const SEDeviceState &getDeviceState();
    bool requestDeviceInfo(DeviceInfo &deviceInfo);

protected:
    StereoCamera();
    inline StereoCameraImpl *getImpl(){return mImpl;}

private:
    StereoCameraImpl *mImpl;
};

#endif // STEREOCAMERA_H
