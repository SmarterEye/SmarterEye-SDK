#include "firmwareupdater.h"
#include <QTimerEvent>
#include <QTimer>
#include <QDebug>
#include <QCryptographicHash>
#include <QFile>
#include <QNetworkDatagram>
#include "protocol.h"
#include "satpext.h"
#include "filesender.h"
#include "message.h"
#include "rtdbkeys.h"
#include "realtimedatabase.h"
#include "sedevicestate.h"
#include "devicestatus.h"
static const int kUpdateProgressCheckInterval = 100;
static const int kCameraErrorLowerValue = 1;
static const int kCameraErrorUpperValue = 9;
static const int kHighTemperatureErrorValue = 19;
static const int kPort = 14628;
static const QStringList kOrionUpdateHardwareVersion={
    "SE0180011","SE0780111"
};

static QMap<QString, int> sProgressMap = {
{"Upgrade progress: extracting package", 10},
{"Upgrade progress: verifying package", 20},
{"Upgrade progress: verifying package successfully.", 30},
{"Upgrade progress: upgrading camera.", 40},
{"Upgrade progress: upgraded camera successfully.", 50},
{"Upgrade progress: upgrading platform.", 60},
{"Upgrade progress: upgraded platform successfully.", 70},
{"Upgrade progress: upgrading application.", 80},
{"Upgrade progress: upgraded application successfully.", 90},
{"Upgrade progress: upgraded all successfully", 100},
};

static QMap<QString, int> sResultMap = {
{"Upgrade error: check package", 1},
{"Upgrade error: can't decrypt the package", 1},
{"Upgrade error: check sum", 1},
{"Upgrade error: error stage", 1},
{"Upgrade error: can't create dir", 2},
{"Upgrade error: extract package", 3},
{"Upgrade error: upgrade application", 4},
{"Upgrade error: upgrade camera", 5},
{"Upgrade error: upgrade platform", 6},
};

FirmwareUpdater::FirmwareUpdater(SATP::Protocol *protocol, RealtimeDatabase *rtdb, QObject *parent):
    QObject(parent),
    mProtocol(protocol),
    mRtdb(rtdb)
{
    protocol->registerBlockHandler(this);
    mFileSender = new SATP::FileSender(protocol, this);
    mFileSender->setAutoDelete(false);
    mUpdateStatus = FirmwareUpdater::Idle;
    mUpgradeProgress = 0;
    mDeviceFailure = false;
    mHighTemperature = false;
    mErrorCode = 0;
    mUpdateWarning = 0;
    mProgressCheckTimerId = -1;
    initSocket();
}

FirmwareUpdater::~FirmwareUpdater()
{
    mProtocol->unregisterBlockHandler(this);
    if(mUdpSocket){
        mUdpSocket->close();
        mUdpSocket->deleteLater();
        mUdpSocket = nullptr;
    }
}

bool FirmwareUpdater::handleReceiveBlock(quint32 dataType, const char *block, int size)
{
    UniqueKey key(dataType);

    switch (key.lowWord()) {
    case DataUnitTypeExt::ResponseUpdate:
        handleUpdateResp(block, size);
        return true;
    }
    return false;
}

void FirmwareUpdater::handleMessage(int type, const char *message, int size)
{
    Q_UNUSED(size)
    handleOldWarningMessage(type,message);
    switch (type) {
    case MessageType::UpdateErrorCode:
    {
        MessageUpdateWarning *msgUpdateWarning = (MessageUpdateWarning *)message;
        mUpdateWarning = msgUpdateWarning->errorCode;
    }
        break;
    }

}

void FirmwareUpdater::handleOldWarningMessage(int type, const char *message)
{
    switch (type) {
    case MessageType::HighTemperatureWarning:
        mHighTemperature = true;
        break;
    case MessageType::DeviceFailureWarning:
    {
        MessageDeviceWarning *msgDeviceWarning = (MessageDeviceWarning*)message;
        mErrorCode = msgDeviceWarning->errCode;
        mDeviceFailure = true;
    }
        break;
    case MessageType::ClearHighTemperatureWarning:
        mHighTemperature = false;
        break;
    case MessageType::ClearDeviceFailureWarning:
        mDeviceFailure = false;
        mErrorCode = 0;
        break;
    case MessageType::ErrorCodeResponse:
    {
        MessageDeviceWarning *msgDeviceWarning = (MessageDeviceWarning*)message;
        mErrorCode = msgDeviceWarning->errCode;
        if(mErrorCode == kHighTemperatureErrorValue && !mHighTemperature){
            mHighTemperature = true;
        }else if(mErrorCode % 10 != 0 && !mDeviceFailure){
            mDeviceFailure = true;
        }
    }
        break;
    }
}

void FirmwareUpdater::handleReady()
{
    mDeviceFailure = false;
    mHighTemperature = false;
    mErrorCode = 0;
    mUpgradeProgress = 0;
    mUpdateStatus = FirmwareUpdater::Idle;
    mUpdateWarning = 0;
    sendMessage(MessageType::ErrorCodeRequest);
    QTimer::singleShot(1000,this,[this]{
        int progress = mRtdb->getValueItem(RtdbKey::DeviceUpdateProgress,0);
        mProductModel = mRtdb->getStringItem(RtdbKey::HardwareVersion);
        qDebug()<<"Camera connected got update progress is:"<<progress;
        handleUpgradeProgress(progress);
    });
}

void FirmwareUpdater::handleReset()
{
    if(mProgressCheckTimerId != -1){
        killTimer(mProgressCheckTimerId);
        mProgressCheckTimerId = -1;
    }
}

void FirmwareUpdater::timerEvent(QTimerEvent *event)
{
    if(event->timerId() == mProgressCheckTimerId){
        if(mUpdateStatus == FirmwareUpdater::Idle){
            killTimer(mProgressCheckTimerId);
            mProgressCheckTimerId = -1;
            mUpgradeProgress = 0;
        }else{
            int progress = mRtdb->getValueItem(RtdbKey::DeviceUpdateProgress,0);
            handleUpgradeProgress(progress);
        }
    }
}

void FirmwareUpdater::startProgressCheckTimer()
{
    QTimer::singleShot(0,this,[this]{
        if(mProgressCheckTimerId == -1 && kOrionUpdateHardwareVersion.indexOf(mProductModel) >= 0){
            mProgressCheckTimerId = startTimer(kUpdateProgressCheckInterval);
        }
    });
}

void FirmwareUpdater::handleSendFileFinished(const QString &fileName)
{
    if(fileName.toLower().contains("firmware")){
        triggerEvent("UploadFinished");
        QString md5 = mFirmwareMd5.toHex();
        mProtocol->sendBlock(DataUnitTypeExt::UpdateFirmware,
                             md5.toUtf8().data(),
                             md5.size());
    }
}

void FirmwareUpdater::handleUpdateResp(const char *block, int size)
{
    Q_UNUSED(size)
    QString resp = QString(block).toLower();
    if(resp.contains("success")){
        triggerEvent("Upload file check finished");
        mUpgradeProgress = 0;
        mUpdateStatus = FirmwareUpdater::Upgrading;
        startProgressCheckTimer();
    }else{
        triggerEvent("Update file check fail finished");
        mUpdateStatus = FirmwareUpdater::Idle;
        mUpgradeProgress = 0;
    }
}

void FirmwareUpdater::handleUpgradeProgress(int progress)
{
    if(progress == 0){
        if(mUpgradeProgress != 0){
            mUpgradeProgress = 0;
            mUpdateStatus = FirmwareUpdater::Idle;
        }
    }else if(progress > 0 && progress <= 100){
        if(mUpdateStatus == FirmwareUpdater::Idle){
            mUpdateStatus = FirmwareUpdater::Upgrading;
            startProgressCheckTimer();
        }
        if(progress >= 99 && mUpgradeProgress < 99){
            mUpgradeProgress = progress;
            triggerEvent("Update success finished");
            QTimer::singleShot(5000,this,[this]{
                mUpgradeProgress = 0;
                mUpdateStatus = FirmwareUpdater::Idle;
            });
        }else{
            mUpgradeProgress = progress;
        }
    }else{
        triggerEvent("Update uncompress fail Finished");
        qDebug()<<"Update warning is:" << mUpdateWarning;
        mUpdateStatus = FirmwareUpdater::Idle;
    }
}

double FirmwareUpdater::getUpdateProgress()
{
    double rate = 0.00;
    if(mUpdateStatus == FirmwareUpdater::Uploading){
        rate = mFileSender->getSendProgress();
    }else if(mUpdateStatus == FirmwareUpdater::Upgrading){
        rate = (double)mUpgradeProgress * 0.01;
    }
    return rate;
}

bool FirmwareUpdater::isDeviceHighTemperature()
{
    return mHighTemperature;
}

int FirmwareUpdater::update(const QString &path)
{
    //1~9:camera error,19:high temperature error
    if(mErrorCode >= kCameraErrorLowerValue && mErrorCode <= kCameraErrorUpperValue){
        triggerEvent("Camera error detected");
        return mErrorCode;
    }
    if(mErrorCode == kHighTemperatureErrorValue || mHighTemperature){
        triggerEvent("Device high temperature");
        return kHighTemperatureErrorValue;
    }

    SEDeviceState state = DeviceStatus::instance()->getDeviceState();
    if(state.errorCode() >= kCameraErrorLowerValue
            && state.errorCode() <= kCameraErrorUpperValue){
        triggerEvent("Camera error detected");
        return state.errorCode();
    }
    if(state.deviceHighTemperature()){
        triggerEvent("Device high temperature");
        return kHighTemperatureErrorValue;
    }

    if (path.isEmpty()) {
        triggerEvent("InvalidFilePath");
        return -1;
    }

    if (mUpdateStatus != FirmwareUpdater::Idle) {
        triggerEvent("DeviceBusy");
        return -2;
    }
    //get md5
    QFile firmware(path);
    if (!firmware.open(QIODevice::ReadOnly)) {
        triggerEvent("OpenFileFailed");
        return -3;
    }

    QCryptographicHash hash(QCryptographicHash::Md5);
    hash.addData(&firmware);
    mFirmwareMd5 = hash.result();
    firmware.close();

    //upload firmware
    mUpdateStatus = FirmwareUpdater::Uploading;
    mFileSender->send(path);

    return 0;
}

void FirmwareUpdater::triggerEvent(const QString &event)
{
    qDebug() << event;
    emit updateEvent(event);
}

int FirmwareUpdater::getUpdateStatus(){
    return mUpdateStatus;
}

void FirmwareUpdater::onErrorOccur(QAbstractSocket::SocketError sockError)
{
    qInfo()<<"socket error occur : "<<sockError;
}

void FirmwareUpdater::processPendingDatagram()
{
    while(mUdpSocket->hasPendingDatagrams()){
        QNetworkDatagram datagram = mUdpSocket->receiveDatagram();
        if(!datagram.isValid()){
            qInfo()<<"datagram is invalid";
        }else {
            decodeUDPdatagram(datagram.data());
        }
    }
}

void FirmwareUpdater::decodeUDPdatagram(QString datagram)
{
    datagram = datagram.trimmed();
    qDebug()<<"receive udp datagram data:"<<datagram;
    if(datagram.contains("Upgrade")){
        triggerEvent("upgrade udp datagram received");
    }
    if(datagram.contains("Upgrade error")){
        mUpdateWarning = sResultMap.value(datagram.mid(0, datagram.lastIndexOf(' ')));
        handleUpgradeProgress(-1);
    }else if(datagram.contains("Upgrade progress")){
        int progress = sProgressMap.value(datagram, 0);
        qDebug()<<"got progress:"<<progress;
        if(mUpdateWarning != 0){
            mUpdateWarning = 0;
        }
        handleUpgradeProgress(progress);
    }
}

void FirmwareUpdater::initSocket()
{
    mUdpSocket = new QUdpSocket(this);
    connect(mUdpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(onErrorOccur(QAbstractSocket::SocketError)));
    if(mUdpSocket->bind(kPort, QUdpSocket::ShareAddress)){
        qInfo()<<"socket bind success";
        connect(mUdpSocket, SIGNAL(readyRead()), this, SLOT(processPendingDatagram()));
    } else {
        qInfo()<<"socket bind error";
    }
}
