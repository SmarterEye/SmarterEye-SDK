#include <thread>
#include <QCoreApplication>
#include <QTimer>
#include <QDebug>
#include <string>
using namespace std;

#include "stereocameraimpl.h"
#include "protocol.h"
#include "connector.h"
#include "framereceiver.h"
#include "filereceiver.h"
#include "camerahandler.h"
#include "messagebus.h"
#include "messageadapter.h"
#include "message.h"
#include "rtdbareceiver.h"
#include "firmwareupdater.h"
#include "taskiddef.h"
#include "rtdbkeys.h"
#include "realtimedatabase.h"
#include "calibrationparams.h"
#include "rotationmatrix.h"
#include "messageutils.h"
#include "QThreadPool"
#include "filesender.h"
#include "deviceinfo.h"

#ifdef Q_OS_LINUX
static const QString kAdasDownloadPath(QString(getenv("HOME")) + "/ADAS_Files/");
#else
static const QString kAdasDownloadPath("D:/");
#endif
// #  define Q_STATIC_ASSERT_X(Condition, Message) static_assert(bool(Condition), Message)
StereoCameraImpl::StereoCameraImpl(QObject *parent)
    : QObject(parent),
    mConnector(nullptr),
    mFrameReceiver(nullptr),
    mFileReceiver(nullptr),
    mMessageBus(nullptr),
    mRtdbReceiver(nullptr),
    mFirmwareUpdater(nullptr),
    mTaskIds(TaskId::AllTasks),
    mMessageAdapter(nullptr),
    mImageWidth(1280),
    mImageHeight(720),
    mLensFocus(8),
    mSensorPixelSize(4.2e-06f),
    mFileSender(nullptr)
{
    QThreadPool::globalInstance()->setMaxThreadCount(20);
    init();
}

StereoCameraImpl::~StereoCameraImpl()
{
    if (mConnector) {
        mConnector->getProtocol()->unregisterBlockHandler(this);
        mMessageBus->unregisterService(this);
        mMessageBus->unregisterService(mRtdbReceiver);
        mMessageBus->unregisterService(mFirmwareUpdater);
        if (mMessageAdapter) {
            mMessageBus->unregisterService(mMessageAdapter);
            delete mMessageAdapter;
        }

        delete mRtdbReceiver;
        delete mFirmwareUpdater;
        delete mFrameReceiver;
        delete mFileReceiver;
        delete mMessageBus;
        mConnector->deleteLater();
    }
    if(mFileSender)delete mFileSender;
}

void StereoCameraImpl::init()
{
    mConnector  = new SATP::Connector();
    SATP::Protocol *protocol = mConnector->getProtocol();
    protocol->registerBlockHandler(this);

    mFrameReceiver = new FrameReceiver(protocol);

    mFileReceiver = new SATP::FileReceiver(protocol);
    mFileReceiver->registerReceiverHandler(this);
    mFileReceiver->setRecvFileDir(kAdasDownloadPath);

    mMessageBus = new MessageBus(false);

    mMessageAdapter = new MessageAdapter(protocol);
    mMessageAdapter->registerUrgentMessage(MessageType::PauseObstacleTask);
    mMessageAdapter->registerUrgentMessage(MessageType::PauseLaneTask);
    mMessageAdapter->registerUrgentMessage(MessageType::PauseDisplayTask);
    mMessageAdapter->registerUrgentMessage(MessageType::PauseHeightTask);

    mMessageBus->registerService(this);
    mRtdbReceiver = new RtdbReceiver(protocol);
    mMessageBus->registerService(mRtdbReceiver);

    mFirmwareUpdater = new FirmwareUpdater(protocol, mRtdbReceiver->getRtdb());
    mMessageBus->registerService(mFirmwareUpdater);
    connect(mFirmwareUpdater,
            SIGNAL(updateEvent(const QString&)),
            this,
            SLOT(onFirmwareUpdateEvent(const QString&)));
    connect(mRtdbReceiver, &RtdbReceiver::rtdbLoaded, this, [this](){
        RealtimeDatabase *rtdb = mRtdbReceiver->getRtdb();
        mImageWidth = rtdb->getValueItem(RtdbKey::ImageWidth, mImageWidth);
        mImageHeight = rtdb->getValueItem(RtdbKey::ImageHeight, mImageHeight);
        mLensFocus = rtdb->getValueItem(RtdbKey::LensFocus, mLensFocus);
        mSensorPixelSize = rtdb->getValueItem(RtdbKey::SensorPixelSize, 4.2f) * 1e-06f;
        emit camImageWidthChanged(mImageWidth);
        emit camImageHeightChanged(mImageHeight);
        emit camLensFocusChanged(mLensFocus);  
        emit camSensorPixelSizeChanged(mSensorPixelSize);
    });
    mMessageBus->registerService(DeviceStatus::instance());
    mIsImuDataEnable = false;
}

void StereoCameraImpl::connectTo(QString addr)
{
    mMessageBus->unregisterService(mMessageAdapter);
    if (!addr.contains("127.0.0.1")) {
        mMessageBus->registerService(mMessageAdapter);
    }
    mConnector->connectTo(addr);

}

SATP::Protocol *StereoCameraImpl::getProtocol()
{
    return mConnector->getProtocol();
}

void StereoCameraImpl::requestFrame(FrameHandler *handler, uint32_t frameIds)
{
    mFrameReceiver->requestFrame(handler, frameIds);
}

bool StereoCameraImpl::handleReceiveBlock(quint32 dataType, const char *block, int size)
{
    Q_UNUSED(dataType)
    Q_UNUSED(block)
    Q_UNUSED(size)
    return false;
}

void StereoCameraImpl::handleReady()
{
    emit cameraConnected();
    controlTasks();
    if(mIsImuDataEnable){
        QTimer::singleShot(1000,this,[this]{
            enableMotionData(true);
        });
    }
}

void StereoCameraImpl::handleReset()
{
    emit cameraDisconnected();
    mRtdbReceiver->getRtdb()->clearData();
}

void StereoCameraImpl::handleReceiveFile(const QString &fileName)
{
    if (fileName.contains("cameraData")) {
        QList<FrameHandler *> handlers = mFrameReceiver->getCameraHandlers();
        for (FrameHandler *handler : handlers) {
            (dynamic_cast<CameraHandler *>(handler))->handleCameraDataFile(fileName.toLocal8Bit().data());
        }
    }
}

void StereoCameraImpl::handleMessage(int type, const char *message, int size)
{
    Q_UNUSED(size)

    switch (type) {
    case MessageType::QueryAvailableFrameIdsResp:
    {
        const MessageIntegerData *msg = reinterpret_cast<const MessageIntegerData*>(message);
        quint16 ids = msg->data;
        emit gotAvailableFrameIds(ids);
        break;
    }

    }
}

void StereoCameraImpl::controlTasks()
{
    struct MessageTaskCommand command;

    command.enable = (mTaskIds & TaskId::ObstacleTask);
    sendMessage(MessageType::PauseObstacleTask, command);

    command.enable = (mTaskIds & TaskId::LaneTask);
    sendMessage(MessageType::PauseLaneTask, command);

    command.enable = (mTaskIds & TaskId::HeightLimitTask);
    sendMessage(MessageType::PauseHeightTask, command);

    command.enable = (mTaskIds & TaskId::DisplayTask);
    sendMessage(MessageType::PauseDisplayTask, command);
}

void StereoCameraImpl::reconnect()
{
    mConnector->reconnect();
}

void StereoCameraImpl::disconnectServer()
{
    mConnector->disconnectServer();
}

bool StereoCameraImpl::isConnected()
{
    return mConnector->getProtocol()->isConnected();
}

QString StereoCameraImpl::getAddress()
{
    return mConnector->getHostName();
}

void StereoCameraImpl::switchStartupMode()
{
    sendMessage(MessageType::SwitchMode);
}

void StereoCameraImpl::enableTasks(uint32_t taskIds)
{
    mTaskIds = taskIds;
    if (isConnected()) {
        controlTasks();
    }
}

void StereoCameraImpl::reboot(bool halt)
{
    MessageShutdown shutdown;
    shutdown.isRebooting = halt ? 0 : 1;
    sendMessage(MessageType::Shutdown, shutdown);
}

int StereoCameraImpl::updateFirmware(QString path)
{
    return mFirmwareUpdater->update(path);
}

double StereoCameraImpl::getUpdateProgress()
{
    return mFirmwareUpdater->getUpdateProgress();
}

int StereoCameraImpl::getUpdateStatus(){
    return mFirmwareUpdater->getUpdateStatus();
}

void StereoCameraImpl::setReceiveDir(const QString &dir)
{
    mFileReceiver->setRecvFileDir(dir);
}

void StereoCameraImpl::onFirmwareUpdateEvent(const QString &event)
{
    QString e = event.toLower();
    Result result;
    if(e.contains("udp")){
        emit udpPackageReceived();
        return;
    }
    if(!e.contains("update")){
        return;
    }

    if(e.contains("success")){
        result.successed = true;
    }else if(e.contains("fail")){
        result.successed = false;
        result.warning = mFirmwareUpdater->getUpdateWarning();
    }
    emit updateFinished(result.successed);
    QList<FrameHandler *> handlers = mFrameReceiver->getCameraHandlers();
    for (FrameHandler *handler : handlers) {
        CameraHandler *camHandler = dynamic_cast<CameraHandler *>(handler);
        if(camHandler)camHandler->handleUpdateFinished(result);
    }
}

bool StereoCameraImpl::requestStereoCameraParameters(StereoCalibrationParameters &params)
{
    if(!isConnected()) {
        return false;
    }
//根据rtdb获取字符串参数
    QString stParamsPart1 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::StereoParametersPart1);
    QString stParamsPart2 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::StereoParametersPart2);
    if(stParamsPart1.isEmpty() || stParamsPart2.isEmpty()) {
        return false;
    }

    QStringList list = (stParamsPart1 + "," + stParamsPart2).split(",");

    params.focus = list[0].toDouble();
    params.cx = list[1].toDouble();
    params.cy = list[2].toDouble();
    params.RRoll = list[3].toDouble();
    params.RPitch = list[4].toDouble();
    params.RYaw = list[5].toDouble();
    params.Tx = list[6].toDouble();
    params.Ty = list[7].toDouble();
    params.Tz = list[8].toDouble();

    return true;
}

bool StereoCameraImpl::requestMonoLeftCameraParameters(MonoCalibrationParameters &params)
{
    if(!isConnected()) {
        return false;
    }

    QString stParamsPart1 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::LeftMonoParametersPart1);
    QString stParamsPart2 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::LeftMonoParametersPart2);
    if(stParamsPart1.isEmpty() || stParamsPart2.isEmpty()) {
        return false;
    }

    QStringList list = (stParamsPart1 + "," + stParamsPart2).split(",");
    params.fx = list[0].toDouble();
    params.fy = list[1].toDouble();
    params.cx = list[2].toDouble();
    params.cy = list[3].toDouble();
    params.k1 = list[4].toDouble();
    params.k2 = list[5].toDouble();
    params.k3 = list[6].toDouble();
    params.p1 = list[7].toDouble();
    params.p2 = list[8].toDouble();

    return true;
}

bool StereoCameraImpl::requestMonoRightCameraParameters(MonoCalibrationParameters &params)
{
    if(!isConnected()) {
        return false;
    }

    QString stParamsPart1 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::RightMonoParametersPart1);
    QString stParamsPart2 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::RightMonoParametersPart2);
    if(stParamsPart1.isEmpty() || stParamsPart2.isEmpty()) {
        return false;
    }

    QStringList list = (stParamsPart1 + "," + stParamsPart2).split(",");
    params.fx = list[0].toDouble();
    params.fy = list[1].toDouble();
    params.cx = list[2].toDouble();
    params.cy = list[3].toDouble();
    params.k1 = list[4].toDouble();
    params.k2 = list[5].toDouble();
    params.k3 = list[6].toDouble();
    params.p1 = list[7].toDouble();
    params.p2 = list[8].toDouble();

    return true;
}

bool StereoCameraImpl::requestRotationMatrix(RotationMatrix &rotationMatrix)
{
    if(!isConnected()) {
        return false;
    }

    QString real3DToImagePart1 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::Real3DToImageRotationMatrixPart1);
    QString real3DToImagePart2 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::Real3DToImageRotationMatrixPart2);
    QString real3DToImagePart3 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::Real3DToImageRotationMatrixPart3);
    QString imageToReal3DPart1 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::ImageToReal3DRotationMatrixPart1);
    QString imageToReal3DPart2 = mRtdbReceiver->getRtdb()->getStringItem(RtdbKey::ImageToReal3DRotationMatrixPart2);
    if(real3DToImagePart1.isEmpty() || real3DToImagePart2.isEmpty() || real3DToImagePart3.isEmpty()
            || imageToReal3DPart1.isEmpty() || imageToReal3DPart2.isEmpty()) {
        return false;
    }

    QStringList list = (real3DToImagePart1 + "," + real3DToImagePart2 + "," + real3DToImagePart3).split(",");
    for(int i=0; i<kNumReal3DToImage; i++) {
        rotationMatrix.real3DToImage[i] = list[i].toFloat();
    }

    list = (imageToReal3DPart1 + "," + imageToReal3DPart2).split(",");
    for(int i=0; i<kNumImageToReal3D; i++) {
        rotationMatrix.imageToReal3D[i] = list[i].toFloat();
    }

    return true;
}

void StereoCameraImpl::enableMaxSendFrameInterval()
{
    mFrameReceiver->enableMaxSendFrameInterval();
}

bool StereoCameraImpl::setFrameRate(float rate)
{
    if (isConnected()) {
        MessageFrameRate command;
        MessageUtils::copyQStringToMString(QString::number(rate), command.frameRate);
        sendMessage(MessageType::SetFrameRate, command);
        return true;
    } else {
        return false;
    }
}

bool StereoCameraImpl::getFrameRate(float &rate)
{
    if (isConnected()) {
        rate = mRtdbReceiver->getRtdb()->getValueItem(RtdbKey::CameraFrameRate, 30.0);
        return true;
    } else {
        return false;
    }
}

bool StereoCameraImpl::getAmbientLight(int &lightness)
{
    if (isConnected()) {
        if(mRtdbReceiver->isLoaded()){
            lightness = mRtdbReceiver->getRtdb()->getValueItem(RtdbKey::EnvIlluminace, 0);
            return true;
        }else{
        return false;
        }
    } else {
        return false;
    }
}

bool StereoCameraImpl::getSmudgeStatus(int &status)
{
    if (isConnected() && mRtdbReceiver->isLoaded()) {
        status = mRtdbReceiver->getRtdb()->getValueItem(RtdbKey::SmudgeStatus, 0);
        return true;
    }
    return false;
}

int StereoCameraImpl::getUpdateWarning()
{
    return mFirmwareUpdater->getUpdateWarning();
}

bool StereoCameraImpl::isDeviceHighTemperature()
{
    bool ret = getDeviceState().deviceHighTemperature()
            || mFirmwareUpdater->isDeviceHighTemperature();
    return ret;
}

void StereoCameraImpl::setImuParameter(const QString &name, int value)
{
    mRtdbReceiver->saveRtdbItemByName(name, value);
}

int StereoCameraImpl::getImuParameter(const QString &name)
{
    return mRtdbReceiver->getRtdbValueByName(name).toInt();
}

bool StereoCameraImpl::sendFileToDevice(const QString &filePath)
{
    if(!mFileSender){
        mFileSender = new SATP::FileSender(mConnector->getProtocol(), this);
    }
    if(!mConnector->isConnected())return false;

    mFileSender->setAutoDelete(false);
    mFileSender->send(filePath);
    return true;
}

void StereoCameraImpl::handleSendFileFinished(const QString &fileName)
{
    emit fileSendFinished(fileName);
}

void StereoCameraImpl::enableMotionData(bool enable)
{
    mRtdbReceiver->saveRtdbItemByName("ImuDataEnable",(quint16)enable);
    mIsImuDataEnable = enable;
}

const SEDeviceState &StereoCameraImpl::getDeviceState()
{
    return DeviceStatus::instance()->getDeviceState();
}

bool StereoCameraImpl::requestDeviceInfo(DeviceInfo &deviceInfo)
{
    if(!isConnected()) {
        return false;
    }

    QString stDeviceInfo1 = mRtdbReceiver->getRtdbValueByName("HardwareVersion").toString();
    QString stDeviceInfo2 = mRtdbReceiver->getRtdbValueByName("SerialNum").toString();
    QString stDeviceInfo3 = mRtdbReceiver->getRtdbValueByName("ProductCode").toString();
    QString stDeviceInfo4 = mRtdbReceiver->getRtdbValueByName("AdasVersion").toString();
    QString stDeviceInfo5 = mRtdbReceiver->getRtdbValueByName("PlatformVersion").toString();
    QString stDeviceInfo6 = mRtdbReceiver->getRtdbValueByName("CameraFirmwareVersion").toString();
    QString stDeviceInfo7 = mRtdbReceiver->getRtdbValueByName("SdkVersion").toString();
    QString stDeviceInfo8 = mRtdbReceiver->getRtdbValueByName("ObsVersion").toString();
    QString stDeviceInfo9 = mRtdbReceiver->getRtdbValueByName("LaneVersion").toString();
    QString stDeviceInfo10 = mRtdbReceiver->getRtdbValueByName("InitialSetupStatus").toString();
    QString stDeviceInfo11 = mRtdbReceiver->getRtdbValueByName("TemperatureCPU").toString();

    if(stDeviceInfo1.isEmpty() || stDeviceInfo2.isEmpty() || stDeviceInfo3.isEmpty()
            || stDeviceInfo4.isEmpty() || stDeviceInfo5.isEmpty() || stDeviceInfo6.isEmpty() ||
            stDeviceInfo7.isEmpty() || stDeviceInfo8.isEmpty() || stDeviceInfo9.isEmpty() || stDeviceInfo10.isEmpty() || stDeviceInfo11.isEmpty()) {
        return false;
    }

    QStringList list = (stDeviceInfo1 + "," + stDeviceInfo2 + "," + stDeviceInfo3 + "," + stDeviceInfo4 + "," + stDeviceInfo5
        + "," + stDeviceInfo6 + "," + stDeviceInfo7 + "," + stDeviceInfo8 + "," + stDeviceInfo9 + "," + stDeviceInfo10  + "," + stDeviceInfo11).split(",");

    deviceInfo.HardwareVersion = list[0].toStdString();
    deviceInfo.SerialNum = list[1].toStdString();
    deviceInfo.ProductCode = list[2].toStdString();
    deviceInfo.AdasVersion = "a" + list[3].toStdString() + "." + mRtdbReceiver->getRtdbValueByName("AdasBuild").toString().toStdString();
    deviceInfo.PlatformVersion = list[4].toStdString();
    deviceInfo.CameraFirmwareVersion = list[5].toStdString();
    deviceInfo.SdkVersion = list[6].toStdString();
    deviceInfo.ObsVersion = list[7].toStdString();
    deviceInfo.LaneVersion = list[8].toStdString();
    deviceInfo.InitialSetupStatus = list[9].toStdString() == "0" ? "uninstalled" : "installed";
    deviceInfo.CPUTemperature =list[10].toStdString();
    return true;
    }
