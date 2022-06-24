#include <thread>
#include <QThread>
#include <QCoreApplication>
#include <QTimer>
#include <QDebug>
#include "stereocamera.h"
#include "stereocameraimpl.h"
//#include "calibrationparams.h"
//#include "rotationmatrix.h"
#include "deviceinfo.h"

StereoCamera::StereoCamera()
{
    mImpl = new StereoCameraImpl();
}

StereoCamera::~StereoCamera()
{
    mImpl->deleteLater();
}

StereoCamera *StereoCamera::connect(const char *addr)
{
    Q_ASSERT( addr != nullptr);
    QCoreApplication *app = QCoreApplication::instance();
    if (app == nullptr) {
        //if used by non-qt application,
        //create thread for event loop.
        std::thread *thread = new std::thread([&app](){
            int argc = 0;
            app = new QCoreApplication(argc, nullptr);
            app->exec();
        });
        Q_UNUSED(thread);
        QThread::msleep(1000); //TODO change to wait condition.
    }

    StereoCamera *camera = nullptr;
    if (QThread::currentThread() != app->thread()) {
        //Create in event loop thread.
        QMetaObject::invokeMethod(
            app,
            [addr, &camera](){
                camera = new StereoCamera();
                camera->getImpl()->connectTo(addr);
            },
            Qt::BlockingQueuedConnection
        );
    } else {
        camera = new StereoCamera();
        camera->getImpl()->connectTo(addr);
    }
    return camera;
}

void StereoCamera::invokeInLoopThread(std::function<void()> method)
{
    QCoreApplication *app = QCoreApplication::instance();
    Q_ASSERT(app);
    if (QThread::currentThread() != app->thread()) {
        //execute in event loop thread.
        QMetaObject::invokeMethod(
            app,
            method,
            Qt::BlockingQueuedConnection
        );
    } else {
        method();
    }
}

SATP::Protocol *StereoCamera::getProtocol()
{
    return mImpl->getProtocol();
}

void StereoCamera::requestFrame(FrameHandler *frameHandler, uint32_t frameIds)
{
    uint32_t x=1<<3;
    x=~x;
    frameIds=frameIds&x;
    std::function<void()> fr = [this, frameHandler, frameIds]{
        mImpl->requestFrame(frameHandler, frameIds);
    };
    invokeInLoopThread(fr);
	
}

bool StereoCamera::requestStereoCameraParameters(StereoCalibrationParameters &params)
{
    return mImpl->requestStereoCameraParameters(params);
}

bool StereoCamera::requestMonoLeftCameraParameters(MonoCalibrationParameters &params)
{
    return mImpl->requestMonoLeftCameraParameters(params);
}

bool StereoCamera::requestMonoRightCameraParameters(MonoCalibrationParameters &params)
{
    return mImpl->requestMonoRightCameraParameters(params);
}

bool StereoCamera::requestRotationMatrix(RotationMatrix &rotationMatrix)
{
    return mImpl->requestRotationMatrix(rotationMatrix);
}

void StereoCamera::enableMaxSendFrameInterval()
{
    std::function<void()> method = [this]{
        mImpl->enableMaxSendFrameInterval();
    };
    invokeInLoopThread(method);
}

bool StereoCamera::setFrameRate(float rate)
{
    bool result = false;
    std::function<void()> method = [this, rate, &result]{
        result = mImpl->setFrameRate(rate);
    };
    invokeInLoopThread(method);

    return result;
}

bool StereoCamera::getFrameRate(float &rate)
{
    return mImpl->getFrameRate(rate);
}

bool StereoCamera::getAmbientLight(int &lightness)
{
    return mImpl->getAmbientLight(lightness);
}

bool StereoCamera::getSmudgeStatus(int &status)
{
    return mImpl->getSmudgeStatus(status);
}

void StereoCamera::switchStartupMode()
{
    std::function<void()> method = [this]{
        mImpl->switchStartupMode();
    };
    invokeInLoopThread(method);
}

void StereoCamera::reboot(bool halt)
{
    std::function<void()> method = [this, halt]{
        mImpl->reboot(halt);
    };
    invokeInLoopThread(method);
}

int StereoCamera::updateFirmware(const char* path)
{
    int result = -1;
    std::function<void()> method = [this, path, &result]{
        result = mImpl->updateFirmware(path);
    };
    invokeInLoopThread(method);

    return result;
}

double StereoCamera::getUpgradeProgress()
{
    double progress = 0;
    if (mImpl->getUpdateStatus() == 1) {
        //in Uploading status
        progress = mImpl->getUpdateProgress() / 2;
    } else if (mImpl->getUpdateStatus() == 2) {
        //in decompress status
        progress = mImpl->getUpdateProgress() / 2 + 0.5;
    }

    return progress;
}

void StereoCamera::setFileReceiveDir(const char *dir)
{
    mImpl->setReceiveDir(dir);
}

void StereoCamera::enableTasks(uint32_t taskIds)
{
    std::function<void()> method = [this, taskIds]{
        mImpl->enableTasks(taskIds);
    };
    invokeInLoopThread(method);
}

void StereoCamera::disconnectFromServer()
{
    std::function<void()> method = [this]{
        mImpl->disconnectServer();
    };
    invokeInLoopThread(method);
}

bool StereoCamera::isConnected()
{
    return getImpl()->isConnected();
}

void StereoCamera::setImuAccelRange(int value)
{
    mImpl->setImuParameter("ImuAccRange",value);
}

void StereoCamera::setImuRotationRange(int value)
{
    mImpl->setImuParameter("ImuGyrRange",value);
}

void StereoCamera::setImuReadFrequence(int value)
{
    mImpl->setImuParameter("ImuSensorOrd",value);
}

int StereoCamera::getImuAccelRange()
{
    return mImpl->getImuParameter("ImuAccRange");
}

int StereoCamera::getImuRotationRange()
{
    return mImpl->getImuParameter("ImuGyrRange");
}

int StereoCamera::getImuReadFrequence()
{
    return mImpl->getImuParameter("ImuSensorOrd");
}

void StereoCamera::enableMotionData(bool enable)
{
    mImpl->enableMotionData(enable);
}

const SEDeviceState &StereoCamera::getDeviceState()
{
    return mImpl->getDeviceState();
}

bool StereoCamera::requestDeviceInfo(DeviceInfo &deviceInfo)
{
    return mImpl->requestDeviceInfo(deviceInfo);
}

bool StereoCamera::sendSetTimeMessage()
{
    return mImpl->sendSetTimeMessage();
}
