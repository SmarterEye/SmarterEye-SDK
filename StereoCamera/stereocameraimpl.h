#ifndef STEREOCAMERAIMPL_H
#define STEREOCAMERAIMPL_H

#include "stereocamera_global.h"
#include <QObject>
#include "blockhandler.h"
#include "filereceiverhandler.h"
#include "filesenderhandler.h"
#include "service.h"
#include "taskiddef.h"
#include "devicestatus.h"
#include "deviceinfo.h"

class Protocol;
class CameraHandler;
class FrameHandler;
class QCoreApplication;
class FrameReceiver;
class MessageBus;
class RtdbReceiver;
class FirmwareUpdater;
struct StereoCalibrationParameters;
struct MonoCalibrationParameters;
class MessageAdapter;
struct RotationMatrix;
struct DeviceInfo;

namespace SATP {
class Protocol;
class FileReceiver;
class Connector;
class FileSender;
}

class STEREOCAMERASHARED_EXPORT StereoCameraImpl : public QObject,
        public SATP::BlockHandler,
        public SATP::FileReceiverHandler,
        public SATP::FileSenderHandler,
        public Service
{
    Q_OBJECT
    Q_PROPERTY(int camImageWidth MEMBER mImageWidth NOTIFY camImageWidthChanged)
    Q_PROPERTY(int camImageHeight MEMBER mImageHeight NOTIFY camImageHeightChanged)
    Q_PROPERTY(int camLensFocus MEMBER mLensFocus NOTIFY camLensFocusChanged)
    Q_PROPERTY(float camSensorPixelSize MEMBER mSensorPixelSize NOTIFY camSensorPixelSizeChanged)
public:
    //public interfaces
    explicit StereoCameraImpl(QObject *parent = nullptr);
    virtual ~StereoCameraImpl() override;
    SATP::Protocol *getProtocol();
    inline SATP::Connector *getConnector(){return mConnector;}
    void requestFrame(FrameHandler *handler, uint32_t frameIds);
    void reboot(bool halt);
    void switchStartupMode();
    void enableTasks(uint32_t taskIds);
    bool requestStereoCameraParameters(StereoCalibrationParameters &params);
    bool requestMonoLeftCameraParameters(MonoCalibrationParameters &params);
    bool requestMonoRightCameraParameters(MonoCalibrationParameters &params);
    bool requestRotationMatrix(RotationMatrix &rotationMatrix);
    void enableMaxSendFrameInterval();
    bool setFrameRate(float rate);
    bool getFrameRate(float &rate);
    uint32_t getActiveTaskIds() {return mTaskIds;}
    bool getAmbientLight(int &lightness);
    bool getSmudgeStatus(int &status);
    void setImuParameter(const QString &name, int value);
    int getImuParameter(const QString &name);
    void enableMotionData(bool enable);
    const SEDeviceState &getDeviceState();
    bool requestDeviceInfo(DeviceInfo &deviceInfo);
    bool sendSetTimeMessage();

    //override BlockHandler
    bool handleReceiveBlock(quint32 dataType, const char *block, int size) override;
    void handleReady() override;
    void handleReset() override;

    //override FileReceiverHandler
    void handleReceiveFile(const QString &fileName) override;
    void handleSendFileFinished(const QString &fileName) override;

    //non-client functions
    inline MessageBus *getMessageBus(){return mMessageBus;}
    inline FrameReceiver *getFrameReceiver(){return mFrameReceiver;}
    inline RtdbReceiver *getRtdbReceiver(){return mRtdbReceiver;}
    inline SATP::FileReceiver *getFileReceiver(){return mFileReceiver;}
    inline FirmwareUpdater *getFirmwareUpdater(){return mFirmwareUpdater;}

signals:
    void cameraConnected();
    void cameraDisconnected();
    void updateFinished(bool successed);
    void gotAvailableFrameIds(quint16 ids);
    void udpPackageReceived();
    void camImageWidthChanged(int width);
    void camImageHeightChanged(int height);
    void camLensFocusChanged(int lens);
    void camSensorPixelSizeChanged(float size);
    void fileSendFinished(const QString &filePath);

public slots:
    void connectTo(QString addr);
    void reconnect();
    void disconnectServer();
    bool isConnected();
    QString getAddress();
    int updateFirmware(QString path);
    double getUpdateProgress();
    int getUpdateStatus();
    void setReceiveDir(const QString &dir);
    int getUpdateWarning();
    bool isDeviceHighTemperature();
    bool sendFileToDevice(const QString &filePath);
protected slots:
    void onFirmwareUpdateEvent(const QString &event);

protected:
    void init();
    void handleMessage(int type, const char *message, int size) override;
    void controlTasks();

private:
    SATP::Connector *mConnector;
    FrameReceiver *mFrameReceiver;
    SATP::FileReceiver *mFileReceiver;
    MessageBus *mMessageBus;
    RtdbReceiver *mRtdbReceiver;
    //for update.
    FirmwareUpdater *mFirmwareUpdater;
    //for tasks
    uint32_t mTaskIds;
    MessageAdapter *mMessageAdapter;
    int mImageWidth;
    int mImageHeight;
    int mLensFocus;
    float mSensorPixelSize;
    SATP::FileSender *mFileSender;
    bool mIsImuDataEnable;
};

#endif // STEREOCAMERAIMPL_H
