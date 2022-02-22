#include "devicestatus.h"
#include "message.h"
#include <QDebug>
DeviceStatus::DeviceStatus(QObject *parent) : QObject(parent)
{
    mIsHighTemperature = false;
    mSeDeviceState = new SEDeviceState();
}

DeviceStatus *DeviceStatus::instance()
{
    static DeviceStatus* inst = nullptr;
    if(!inst){
        inst = new DeviceStatus;
        Q_ASSERT(inst);
    }
    return inst;
}

void DeviceStatus::handleMessage(int type, const char *message, int size)
{
    Q_UNUSED(size)
    QMutexLocker locker(&mAccessMutex);
    handleOldWarningMessage(type,message);
    switch (type) {
    case MessageType::DeviceWarningInfo:
    {
        MessageDeviceWarning *msgDeviceWarning = (MessageDeviceWarning*)message;
        mSeDeviceState->setErrorCodeList(msgDeviceWarning->errCodeList);
        if(msgDeviceWarning->errCode != 0){
            mSeDeviceState->setDeviceState(SEDeviceState::AbnormalState);
        }else{
            mSeDeviceState->setDeviceState(SEDeviceState::NormalState);
        }
    }
        break;
    }
}

void DeviceStatus::handleOldWarningMessage(int type, const char *message)
{
    switch (type) {
    case MessageType::DeviceFailureWarning:
    {
        MessageDeviceWarning *msgDeviceWarning = (MessageDeviceWarning*)message;
        mSeDeviceState->setErrorCodeList(msgDeviceWarning->errCodeList);
        mSeDeviceState->setDeviceState(SEDeviceState::AbnormalState);
    }
        break;
    case MessageType::ClearDeviceFailureWarning:
        mSeDeviceState->clearErrorCodeList();
        if(mIsHighTemperature){
            mSeDeviceState->setDeviceState(SEDeviceState::HighTemperatureState);
        }else{
            mSeDeviceState->setDeviceState(SEDeviceState::NormalState);
        }
        break;
    case MessageType::HighTemperatureWarning:
        mIsHighTemperature = true;
        if(mSeDeviceState->errorCode()== 0) mSeDeviceState->setDeviceState(SEDeviceState::HighTemperatureState);
        break;
    case MessageType::ClearHighTemperatureWarning:
        mIsHighTemperature = false;
        if(mSeDeviceState->errorCode()== 0) mSeDeviceState->setDeviceState(SEDeviceState::NormalState);
        break;
    }
}

const SEDeviceState &DeviceStatus::getDeviceState()
{
    QMutexLocker locker(&mAccessMutex);
    return *mSeDeviceState;
}
