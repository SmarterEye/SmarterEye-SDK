#include <QByteArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>
#include <QObject>
#include <QFile>
#include <QDir>
#include <QDateTime>
#include <QTimer>
#include <QDebug>
#include "rtdbmanager.h"
#include "messagebus.h"
#include "message.h"
#include "messageutils.h"
#include "rtdbkeys.h"
#include "savetodisk.h"

QString RtdbManager::offlineImagePath()
{
#ifdef Q_OS_LINUX
    return "/mnt/adasUserData/roadImage";
#else
    return "D:/ADAS/mnt/adasUserData/Config";
#endif
}

RtdbManager::RtdbManager()
{
    mRtdb = new RealtimeDatabase();
    mIsShutdown = false;
    mProjectSettings = nullptr;
}

RtdbManager::~RtdbManager()
{
    cleanup();
}

void RtdbManager::init(QString deviceInfoFile, QString cameraInfoFile)
{
    mDeviceInfoFile = deviceInfoFile;
    mCameraInfoFile = cameraInfoFile;
    initProjectSettings();
    int ret = initRtdbItems();
    Q_ASSERT(ret);
    mSavingTimerId = startTimer(3000);
}

bool RtdbManager::initRtdbItems()
{
    readDefaultItems();
    restoreItemsFromSettings();

    bool restored = restoreItemsFromStorge(mDeviceInfoFile);
    restoreItemsFromStorge(mCameraInfoFile, true);

    if (!addItemsToRtdb()) {
        return false;
    }
    if (!restored) {
        save();
    }
    return true;
}

bool RtdbManager::readDefaultItems()
{
    mRtdbItems
        << new RtdbItem{RtdbKey::AdasVersion, RealtimeDatabase::STRING, "AdasVersion", "1.0.0", false}
        << new RtdbItem{RtdbKey::AdasBuild, RealtimeDatabase::INT64, "AdasBuild", 1000, false}
        << new RtdbItem{RtdbKey::VideoSource, RealtimeDatabase::STRING, "VideoSource", "stereo", true}
        << new RtdbItem{RtdbKey::DownSampleType, RealtimeDatabase::INT32, "DownSampleType", 8, true}
        << new RtdbItem{RtdbKey::CameraFrameRate, RealtimeDatabase::DOUBLE, "CameraFrameRate", 30, true}
        << new RtdbItem{RtdbKey::OfflineImagePath, RealtimeDatabase::STRING, "OfflineImagePath", offlineImagePath(), true}

        << new RtdbItem{RtdbKey::BuildConfig, RealtimeDatabase::STRING, "BuildConfig", "PRD", false}
        << new RtdbItem{RtdbKey::ProductCode, RealtimeDatabase::STRING, "ProductCode", "XXX", false}
        << new RtdbItem{RtdbKey::CollisionAlarmGranularity, RealtimeDatabase::UINT16, "CollisionAlarmGranularity", 1, true}
        << new RtdbItem{RtdbKey::AlarmVolume, RealtimeDatabase::UINT16, "AlarmVolume", 80, true}
        << new RtdbItem{RtdbKey::LaneDepartureAlarmGranularity, RealtimeDatabase::UINT16, "LaneDepartureAlarmGranularity", 1, true}

        << new RtdbItem{RtdbKey::TaskRunnerStatus, RealtimeDatabase::INT64, "TaskRunnerStatus", 1, false}
        << new RtdbItem{RtdbKey::TaskRunnerMemoryUsage, RealtimeDatabase::INT64, "TaskRunnerMemoryUsage", 1, false}
        << new RtdbItem{RtdbKey::LaneTaskStatus, RealtimeDatabase::INT64, "LaneTaskStatus", 1, false}
        << new RtdbItem{RtdbKey::LaneTaskAverage, RealtimeDatabase::INT64, "LaneTaskAverage", 1, false}
        << new RtdbItem{RtdbKey::LaneTaskFrameRate, RealtimeDatabase::INT64, "LaneTaskFrameRate", 1, false}
        << new RtdbItem{RtdbKey::ObstacleExtractionTaskAverage, RealtimeDatabase::INT64, "ObstacleExtractionTaskAverage", 1, false}
        << new RtdbItem{RtdbKey::ObstacleExtractionTaskFrameRate, RealtimeDatabase::INT64, "ObstacleExtractionTaskFrameRate", 1, false}
        << new RtdbItem{RtdbKey::DaemonStatus, RealtimeDatabase::INT64, "DaemonStatus", 1, false}

        << new RtdbItem{RtdbKey::MemoryUsage, RealtimeDatabase::UINT16, "MemoryUsage", 1, false}
        << new RtdbItem{RtdbKey::CpuUsage, RealtimeDatabase::UINT16, "CpuUsage", 1, false}
        << new RtdbItem{RtdbKey::Speed, RealtimeDatabase::DOUBLE, "Speed", 0, false}
        << new RtdbItem{RtdbKey::SysStartupMode, RealtimeDatabase::STRING, "SysStartupMode", "normal", true}
        << new RtdbItem{RtdbKey::ObstacleAlgType, RealtimeDatabase::INT32, "ObstacleAlgType", 0, false}
        << new RtdbItem{RtdbKey::InvalidObstacleFilter, RealtimeDatabase::INT32, "InvalidObstacleFilter", 1, true}

        << new RtdbItem{RtdbKey::TemperatureCPU, RealtimeDatabase::UINT16, "TemperatureCPU", 1, false}

        << new RtdbItem{RtdbKey::Model, RealtimeDatabase::STRING, "Model", "C1", false}
        << new RtdbItem{RtdbKey::CameraType, RealtimeDatabase::STRING, "CameraType", "remap", true}
        << new RtdbItem{RtdbKey::InstallType, RealtimeDatabase::INT16, "InstallType", 0, false}
        << new RtdbItem{RtdbKey::PlatformVersion, RealtimeDatabase::STRING, "PlatformVersion", "", false}
        << new RtdbItem{RtdbKey::CameraFirmwareVersion, RealtimeDatabase::STRING, "CameraFirmwareVersion", "", false}
        << new RtdbItem{RtdbKey::InitialSetupStatus, RealtimeDatabase::INT64, "InitialSetupStatus", 0, true}
        << new RtdbItem{RtdbKey::LaneLearningMode, RealtimeDatabase::STRING, "LaneLearningMode", "unknown", true}
        << new RtdbItem{RtdbKey::MaxLaneTaskProcessRate, RealtimeDatabase::INT64, "MaxLaneTaskProcessRate", 25, true}
        << new RtdbItem{RtdbKey::MaxSendFrameInterval, RealtimeDatabase::INT64, "MaxSendFrameInterval", 80, true}
        << new RtdbItem{RtdbKey::AlarmDevice, RealtimeDatabase::UINT16, "AlarmDevice", 0, true}
        << new RtdbItem{RtdbKey::AlarmAdjustEnable, RealtimeDatabase::UINT16, "AlarmAdjustEnable", 1, true}
        << new RtdbItem{RtdbKey::SystemVolumeLevel, RealtimeDatabase::UINT16, "SystemVolumeLevel", 3, true}
        << new RtdbItem{RtdbKey::FcwWorkingThreshold, RealtimeDatabase::UINT16, "FcwWorkingThreshold", 10, true}
        << new RtdbItem{RtdbKey::FcwAlarmThreshold, RealtimeDatabase::STRING, "FcwAlarmThreshold", "2800,1500", true}
        << new RtdbItem{RtdbKey::LdwWorkingThreshold, RealtimeDatabase::UINT16, "LdwTaskWorkingSpeed", 50, true}
        << new RtdbItem{RtdbKey::LdwAdjustEnable, RealtimeDatabase::UINT16, "LdwAdjustEnable", 1, true}
        << new RtdbItem{RtdbKey::HmwWorkingThreshold, RealtimeDatabase::UINT16, "HmwWorkingThreshold", 10, true}
        << new RtdbItem{RtdbKey::HmwAdjustEnable, RealtimeDatabase::UINT16, "HmwAdjustEnable", 1, true}
        << new RtdbItem{RtdbKey::HmwAlarmThreshold, RealtimeDatabase::INT64, "HmwAlarmThreshold", 1000, true}
        << new RtdbItem{RtdbKey::SerialNum, RealtimeDatabase::STRING, "SerialNum", "", false}
        << new RtdbItem{RtdbKey::DriveAreaPoints, RealtimeDatabase::STRING, "DriveAreaPoints", "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,640,0,640,719,0,0,0,0,0,360,1279,360", false}
        << new RtdbItem{RtdbKey::SttpPollInterval, RealtimeDatabase::INT64, "SttpPollInterval", 50, true}
        << new RtdbItem{RtdbKey::UserName, RealtimeDatabase::STRING, "UserName", "customer", false}
        << new RtdbItem{RtdbKey::OffsetDistance, RealtimeDatabase::INT64, "OffsetDistance", 3000, true}
        << new RtdbItem{RtdbKey::SafetyDistance, RealtimeDatabase::INT64, "SafetyDistance", 0, true}
        << new RtdbItem{RtdbKey::ObsVersion, RealtimeDatabase::STRING, "ObsVersion", "", false}
        << new RtdbItem{RtdbKey::LaneVersion, RealtimeDatabase::STRING, "LaneVersion", "", false}
        << new RtdbItem{RtdbKey::SdkVersion, RealtimeDatabase::STRING, "SdkVersion", "", false}
        << new RtdbItem{RtdbKey::ThirdDeviceInterface, RealtimeDatabase::STRING, "ThirdDeviceInterface", "", true}
        << new RtdbItem{RtdbKey::SpeedInterface, RealtimeDatabase::STRING, "SpeedInterface", "CAN", true}
        << new RtdbItem{RtdbKey::TurnSignalInterface, RealtimeDatabase::STRING, "TurnSignalInterface", "", true}
        << new RtdbItem{RtdbKey::WiperInterface, RealtimeDatabase::STRING, "WiperInterface", "", true}
        << new RtdbItem{RtdbKey::BrakeInterface, RealtimeDatabase::STRING, "BrakeInterface", "", true}
        << new RtdbItem{RtdbKey::ScreenModel, RealtimeDatabase::STRING, "ScreenModel", "", true}
        << new RtdbItem{RtdbKey::ScreenSN, RealtimeDatabase::STRING, "ScreenSN", "", true}
        << new RtdbItem{RtdbKey::ScreenVersion, RealtimeDatabase::STRING, "ScreenVersion", "", true}
        << new RtdbItem{RtdbKey::Distance2Ground, RealtimeDatabase::INT64, "Distance2Ground", 0, false}
        << new RtdbItem{RtdbKey::Distance2LeftGlass, RealtimeDatabase::INT64, "Distance2LeftGlass", 0, false}
        << new RtdbItem{RtdbKey::Distance2RightGlass, RealtimeDatabase::INT64, "Distance2RightGlass", 0, false}
        << new RtdbItem{RtdbKey::Distance2Bumper, RealtimeDatabase::INT64, "Distance2Bumper", 0, false}
        << new RtdbItem{RtdbKey::WheelGap, RealtimeDatabase::INT64, "WheelGap", 0, false}
        << new RtdbItem{RtdbKey::CarHeadHeight, RealtimeDatabase::INT64, "CarHeadHeight", 0, false}
        << new RtdbItem{RtdbKey::DistanceBumper2Axle, RealtimeDatabase::INT64, "DistanceBumper2Axle", 0, false}
        << new RtdbItem{RtdbKey::VehicleBrand, RealtimeDatabase::STRING, "VehicleBrand", "", false}
        << new RtdbItem{RtdbKey::VehicleModel, RealtimeDatabase::STRING, "VehicleModel", "", false}
        << new RtdbItem{RtdbKey::ManufactureDate, RealtimeDatabase::STRING, "ManufactureDate", "", false}
        << new RtdbItem{RtdbKey::LicenseNumber, RealtimeDatabase::STRING, "LicenseNumber", "", false}
        << new RtdbItem{RtdbKey::HardwareVersion, RealtimeDatabase::STRING, "HardwareVersion", "", false}
        << new RtdbItem{RtdbKey::SystemVersion, RealtimeDatabase::STRING, "SystemVersion", "", false}
        << new RtdbItem{RtdbKey::CorrectedDeviceInstallHeight, RealtimeDatabase::INT64, "CorrectedDeviceInstallHeight", 0, false}
        << new RtdbItem{RtdbKey::ObsSamplePeriod, RealtimeDatabase::UINT16, "ObsSamplePeriod", 6, true}
        << new RtdbItem{RtdbKey::WinOfflinePath, RealtimeDatabase::STRING, "WinOfflinePath", offlineImagePath(), true}
        << new RtdbItem{RtdbKey::StereoParametersPart1, RealtimeDatabase::STRING, "StereoParametersPart1", "0,0,0,0,", false}
        << new RtdbItem{RtdbKey::StereoParametersPart2, RealtimeDatabase::STRING, "StereoParametersPart2", "0,0,0,0,0,", false}
        << new RtdbItem{RtdbKey::LeftMonoParametersPart1, RealtimeDatabase::STRING, "LeftMonoParametersPart1", "0,0,0,0,", false}
        << new RtdbItem{RtdbKey::LeftMonoParametersPart2, RealtimeDatabase::STRING, "LeftMonoParametersPart2", "0,0,0,0,0,", false}
        << new RtdbItem{RtdbKey::RightMonoParametersPart1, RealtimeDatabase::STRING, "RightMonoParametersPart1", "0,0,0,0,", false}
        << new RtdbItem{RtdbKey::RightMonoParametersPart2, RealtimeDatabase::STRING, "RightMonoParametersPart2", "0,0,0,0,0,", false}
        << new RtdbItem{RtdbKey::ImageWidth, RealtimeDatabase::UINT16, "ImageWidth", 1280, false}
        << new RtdbItem{RtdbKey::ImageHeight, RealtimeDatabase::UINT16, "ImageHeight", 720, false}
        << new RtdbItem{RtdbKey::LensFocus, RealtimeDatabase::UINT16, "LensFocus", 8, false}
        << new RtdbItem{RtdbKey::EnvIlluminace, RealtimeDatabase::UINT16, "EnvIlluminace", 0, false}
        << new RtdbItem{RtdbKey::SmudgeStatus, RealtimeDatabase::UINT16, "SmudgeStatus", 2, false}
        << new RtdbItem{RtdbKey::LaneLearningProgress, RealtimeDatabase::INT64, "LaneLearningProgress", 0, false}

        << new RtdbItem{RtdbKey::HeightLimitTaskAverage, RealtimeDatabase::INT64, "HeightLimitTaskAverage", 1, false}
        << new RtdbItem{RtdbKey::HeightLimitTaskFrameRate, RealtimeDatabase::INT64, "HeightLimitTaskFrameRate", 1, false}
        << new RtdbItem{RtdbKey::HeightVersion, RealtimeDatabase::STRING, "HeightVersion", "", false}

        << new RtdbItem{RtdbKey::Real3DToImageRotationMatrixPart1, RealtimeDatabase::STRING, "Real3DToImageRotationMatrixPart1", "0,0,0,0,", false}
        << new RtdbItem{RtdbKey::ImageToReal3DRotationMatrixPart1, RealtimeDatabase::STRING, "ImageToReal3DRotationMatrixPart1", "0,0,0,0,", false}
        << new RtdbItem{RtdbKey::DeviceUpdateProgress, RealtimeDatabase::INT16, "DeviceUpdateProgress", 0, false}
        << new RtdbItem{RtdbKey::Real3DToImageRotationMatrixPart2, RealtimeDatabase::STRING, "Real3DToImageRotationMatrixPart2", "0,0,0,0,", false}
        << new RtdbItem{RtdbKey::Real3DToImageRotationMatrixPart3, RealtimeDatabase::STRING, "Real3DToImageRotationMatrixPart3", "0,0,0,0,", false}
        << new RtdbItem{RtdbKey::ImageToReal3DRotationMatrixPart2, RealtimeDatabase::STRING, "ImageToReal3DRotationMatrixPart2", "0,0,0,0,0,", false}
        << new RtdbItem{RtdbKey::CanProtocol, RealtimeDatabase::STRING, "CanProtocol", "", true}

        << new RtdbItem{RtdbKey::SensorPixelSize, RealtimeDatabase::DOUBLE, "SensorPixelSize", 4.2, false}
        << new RtdbItem{RtdbKey::BinocularBaseline, RealtimeDatabase::DOUBLE, "BinocularBaseline", 120, false}
        << new RtdbItem{RtdbKey::CameraCalibStatus, RealtimeDatabase::UINT16, "CameraCalibStatus", 0, false}
        << new RtdbItem{RtdbKey::UpperCPUTemperature, RealtimeDatabase::INT16, "UpperCPUTemperature", 100, false}
        << new RtdbItem{RtdbKey::LengthDetectionUpperValue, RealtimeDatabase::DOUBLE, "LengthDetectionUpperValue", 30, true}
        << new RtdbItem{RtdbKey::PrimaryWarningDistance, RealtimeDatabase::DOUBLE, "PrimaryWarningDistance", 25, true}
        << new RtdbItem{RtdbKey::AdvanceWarningDistance, RealtimeDatabase::DOUBLE, "AdvanceWarningDistance", 10, true}
        << new RtdbItem{RtdbKey::ObsWarningVersion, RealtimeDatabase::STRING, "ObsWarningVersion", "", false}
        << new RtdbItem{RtdbKey::LensSmudgeVersion, RealtimeDatabase::STRING, "LensSmudgeVersion", "", false}
        << new RtdbItem{RtdbKey::ScreenLogoID, RealtimeDatabase::STRING, "ScreenLogoID", "1", false}
        << new RtdbItem{RtdbKey::ImuAccRange, RealtimeDatabase::INT16, "ImuAccRange", 4, true}
        << new RtdbItem{RtdbKey::ImuGyrRange, RealtimeDatabase::INT16, "ImuGyrRange", 500, true}
        << new RtdbItem{RtdbKey::ImuSensorOrd, RealtimeDatabase::INT16, "ImuSensorOrd", 100, true}
        << new RtdbItem{RtdbKey::ImuPosture, RealtimeDatabase::INT16, "ImuPosture", 1, true}
        << new RtdbItem{RtdbKey::ImuDataEnable, RealtimeDatabase::INT16, "ImuDataEnable", 0, false}

        << new RtdbItem{RtdbKey::CameraSN, RealtimeDatabase::STRING, "CameraSN", "", false}
        << new RtdbItem{RtdbKey::CameraPN, RealtimeDatabase::STRING, "CameraPN", "", false}
        << new RtdbItem{RtdbKey::YawRateInterface, RealtimeDatabase::STRING, "YawRateInterface", "", true}
        << new RtdbItem{RtdbKey::HardwareNumber, RealtimeDatabase::STRING, "HardwareNumber", "", false}
        << new RtdbItem{RtdbKey::GearInterface, RealtimeDatabase::STRING, "GearInterface", "", true}
           ;

    return true;
}

bool RtdbManager::restoreItemsFromStorge(QString filePath, bool readAll)
{
    QFile devicePropertyFile(filePath);
    if (!devicePropertyFile.open(QIODevice::ReadOnly)) {
        qDebug() << "Couldn't open file: " << filePath;
        return false;
    }
    QByteArray devicePropertyData = devicePropertyFile.readAll();
    devicePropertyFile.close();
    QJsonDocument devicePropertyDoc(QJsonDocument::fromJson(devicePropertyData));
    QJsonObject object;
    object=devicePropertyDoc.object();

    QJsonValue setting =object.value("settings");

    if(setting.type() == QJsonValue::Undefined) return false;

    QJsonObject settingObj = setting.toObject();

    for (RtdbItem *item : mRtdbItems) {
        if (!item->persistent && !readAll) continue;
        QJsonValue jVal = settingObj.value(item->name);
        if (QJsonValue::Undefined != jVal.type()) {
            item->value = jVal.toVariant();
        }
    }

    return true;
}
bool RtdbManager::restoreItemsFromSettings()
{

    for (RtdbItem *item : mRtdbItems) {
        if(!mProjectSettings->contains(item->name))continue;
        item->value = mProjectSettings->value(item->name);
    }

    return true;
}

bool RtdbManager::addItemsToRtdb()
{
    for (RtdbItem *item : mRtdbItems) {
        mRtdb->addItem(*item);
    }

    return mRtdb->create();
}

void RtdbManager::cleanup()
{
    killTimer(mSavingTimerId);
    delete mRtdb;
}

void RtdbManager::timerEvent(QTimerEvent *event)
{
    Q_UNUSED(event)
}

bool RtdbManager::save()
{
    if (mIsShutdown) return true;

    bool result = true;

    QJsonObject settingObject;
    for (RtdbItem *item : mRtdbItems) {
        if (item->persistent) {
            RealtimeDatabase::RtdbItem rtdbItem;
            if (mRtdb->getItem(item->key, rtdbItem)) {
                settingObject.insert(item->name, rtdbItem.value.toString());
            }
        }
    }

    QJsonObject object;
    object.insert("settings",settingObject);
    QJsonDocument loadDoc(object);
    QFile loadFile(mDeviceInfoFile);
    if (!loadFile.open(QIODevice::WriteOnly)) {
        qWarning("Couldn't open deviceInfo file.");
        result = false;
        return result;
    }

    int ret = loadFile.write(loadDoc.toJson(QJsonDocument::Compact));
    if(ret == -1) result = false;
    loadFile.close();
    saveToDisk();
    return result;
}

//change memory data acrodding to message
void RtdbManager::handleMessage(int type, const char *message, int size)
{
    Q_UNUSED(size)

    switch (type) {
    case MessageType::Shutdown:
        mIsShutdown = true;
        break;
    case MessageType::RtdbChanged:
    {
        const MessageRtdbChanged *msgRtdbChanged = (const MessageRtdbChanged *)message;
        RealtimeDatabase::RtdbItem rtdbItem;
        if (mRtdb->getItem(msgRtdbChanged->changedKey, rtdbItem)) {
            if (rtdbItem.persistent) {
                save();
            }
        }
    }
        break;
    default:
        break;
    }
}

void RtdbManager::initProjectSettings()
{
    QSettings::setPath(QSettings::NativeFormat, QSettings::SystemScope,
                           "/usr/local/config");
    if(!mProjectSettings){
        mProjectSettings = new QSettings(QSettings::NativeFormat,
                                      QSettings::SystemScope,
                                      "SmarterEye", "deviceSettings");
    }
}
