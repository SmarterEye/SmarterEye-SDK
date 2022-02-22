#ifdef SMARTMESSAGE_LIBRARY
#undef MESSAGEDEF
#endif

#ifndef MESSAGEDEF
#define MESSAGEDEF

#ifndef SMARTMESSAGE_LIBRARY
#include <qglobal.h>
#include <QChar>
#include <QString>
#include "taskiddef.h"

#pragma pack(push, 1)

static const int kGeneralLogLength = 512;

struct MessageType
{
    enum Enumeration {
#else
    enum MessageTypeDef{
#endif
        None  = 0,
        All,
        PlayAudio,
        Speed,
        Capture,
        CaptureBegan,
        CaptureFinished,
        Snapshot,
        SnapshotResp,
        GneralLog,
        CameraOpen,
        CameraClose,
        VideoPause,
        VideoResume,
        Shutdown,

        LoadCameraData,
        AutoMark,
        PlayShutterSound,
        ExtendAdasInfo,
        ExtendObstacleInfo,
        ExtendLaneInfo,
        VideoSourceAvailable,
        FrontVehicleAlarm,
        Obsolete_LaneDepartureAlarmGranularity,
        Obsolete_LaneDepartureAlarmWay,
        AdasInfoReq,
        AdasInfoResp,
        Net4GSignalIndensity,
        CollisionWarningLog,    //collision warning data report log message
        RtdbChanged,
        DeviceWarningInfo,
        SaveRtdb,
        CollisionAlarmManual,
        LaneDapartureAlarmManual,
        FactoryMarkReq,
        FactoryMarkResp,
        LeftFrameReq,
        LeftFrameResp,
        RightFrameReq,
        RightFrameResp,
        Obsolete_SaveTripMiles,
        UpgradeMD5Check,
        SwitchMode,
        ResetRoadModel,
        ResetCollisionArea,
        FileTransferResult,
        SetTime,
        FactoryRestore,
        CollisionAlarm,
        LaneDapartureAlarm,
        Set25FPS,
        Set30FPS,
        GpioCommand,
        GpioEvent,
        GpioRequest,
        GpioResponse,
        BrakeEnable,
        BrakeDismiss,
        ObdSleep,
        SwapToWifi,
        SwapToHotspot,
        RestartTaskRunner,
        GPSSignalIdensity,
        ResizeCalibrationScale,
        PrimaryMarkAudio = 511, //C2 messages definition
        CamFrameReq = 512,
        ScaleImgReq,
        PossitionData,
        GPSSpeed,
        PauseObstacleTask, // todo: discard
        PauseLaneTask, // todo: discard
        GetD2DTableReq,
        GetD2DTableResp,
        GetIpAddrReq,
        GetIpAddrResp,
        SetIpAddrReq,
        SetIpAddrResp,
        GetTransferRateReq,
        GetTransferRateResp,
        SetTransferRateReq,
        SetTransferRateResp,
        ErrorReport,
        AlarmVolumeChanged,
        TaskStatusChanged,
        ImuInstallCalibReq,
        ImuInstallCalibResp,
        GetUserFileReq,
        GetUserFileResp,
        AlarmVolume,
        SwitchToAutoLaneMark,
        SwitchToManualLaneMark,
        LaneMarkSucceedResp,
        CameraInstallParam,
        SetSensorOffsetReq,
        SetSensorOffsetResp,
        AutoMarkResult,
        GetCameraTypeReq,
        GetCameraTypeResp,
        VehicleStatusReport,
        KeySound,
        SoftShutdown,
        HighTemperatureWarning,
        ClearHighTemperatureWarning,
        DeviceFailureWarning,
        ClearDeviceFailureWarning,
        Obsolete_BackToNoramlState,
        LaneLearning,
        SaveCanProtocolContent,
        LaneDapartureAlarmScreen,
        CollisionAlarmScreen,
        BeepSound,
        CanConfigrationReq,
        VehicleEvent,
        CollisionAlarmBeep,
        PauseDisplayTask, // todo: discard
        GetScreenSystemInfo,
        SetScreenSystemInfo,
        ReConfigureScreenMenu,
        OutdoorCalibCheckReq,
        OutdoorCalibCheckResp,
        turnSignalTestReq,
        turnSignalTestResp,
        laneStudyAgain,
        originDepthCalibReq,
        QueryAvailableFrameIdsReq,
        QueryAvailableFrameIdsResp,
        Obsolete_ReportMotionData,
        PauseHeightTask, // todo: discard
        CanProtocolTestReq,
        CanProtocolTestResp,
        UpdateErrorCode,
        ErrorCodeRequest,
        ErrorCodeResponse,
        Prepare4Calib,
        DeviceStandby,
        DeviceActive,
        SumdgeDetectWarning,
        ClearSumdgeWarning,
        SetFrameRate,
        ControlTasks    // for replacing PauseTask messages
    };
#ifdef SMARTMESSAGE_LIBRARY
    Q_ENUM(MessageTypeDef)
#else
    };
struct LogType {
    enum Enumeration {
        Debug,
        Information,
        Error,
        Warrning
    };
};

struct MessageHead{
    qint64 time;
};

struct MessageNull{
    MessageHead head;
};

struct MessageString{
    quint16 length;
    QChar data[kGeneralLogLength];//encoded as unicode
};

struct MessageSpeed{
    MessageHead head;
    double speed;
    double acceleration;
};

struct MessageRtdbChanged{
    MessageHead head;
    quint16 changedKey;
};

struct MessagePlayAudio{
    MessageHead head;
    MessageString fileName;
};

struct MessageAlarmVolume{
    MessageHead head;
    quint16 value;//range:0~100
};

struct MessageGneralLog{
    MessageHead head;
    quint16 tag;
    MessageString log;
};

struct MessageSnapshotResp{
    MessageHead head;
    MessageString files;
};

struct MessageCaptureResp{
    MessageHead head;
    quint16 status; //1: On , 0: Off
};

struct MessageSettingADASINFO{
    MessageHead head;
    MessageString adasInfo;
};

struct MessageSettingRESP{
    MessageHead head;
    quint16 status; //0: success , 1: error
};

struct MessageUpgradeMD5Check{
    MessageHead head;
    bool value;
};

struct MessageShutdown{
    MessageHead head;
    bool    isRebooting; //shutdown , reboot
    quint16 delayMS;
    MessageShutdown(bool isRebooting_ = false, quint16 delayMS_ = 100):
        isRebooting(isRebooting_),
        delayMS(delayMS_)
    {

    }
};

struct MessageFactoryMarkRESP{
    MessageHead head;
    quint16 value; //range:0~25
};

struct MessageCollisionAlarm{
    enum CollisionAlarmType {
        HMW = 0,
        FCW,
        NoWarning,
        HLC
    };
    MessageHead head;
    CollisionAlarmType warningType;
    int speed;
    float relativeSpeed;
    float hmw;
    int level;
};

struct MessageCamFrameReq{
    MessageHead head;
    quint16 frameId;
};

struct MessagePossitionData{
    MessageHead head;
    qreal latitude; //positive is for North and negative is for South.
    qreal longitude;//positive is for East and negative is for West.
    qreal altitude;
};

struct MessageSetTime{
    MessageHead head;
    qint64 value;
};

struct MessageFactoryMarkAudio{
    MessageHead head;
    qint16 value; //0:snapshot 1:success 2:faild
};

struct MessageTaskCommand{
    MessageHead head;
    quint16 enable;//0:disable , 1:enable.
    quint16 taskId;
};

struct MessageIpAddress{
    MessageHead head;
    quint32 ip;
    quint32 mask;
    quint32 gateway;
};

struct MessageFloatData{
    MessageHead head;
    float data;
};
struct MessageIntegerData{
    MessageHead head;
    int data;
};

struct MessageTaskStatusChanged{
    MessageHead head;
    MessageString taskName;
    int status;
};

struct MessageFileNames{
    MessageHead head;
    MessageString files;
};

struct MessageSwitchToManualLaneMark
{
    MessageHead head;
    bool confirmed;
};

struct MessageCameraInstallParam
{
    MessageHead head;
    int cameraHeight;
    int leftDistance;
    int rightDistance;
    int depth;
    int carWidth;
    int carHeadHeight;
    int bumper2Axle;
};

struct MessageGetSysStartupModeResp
{
    MessageHead head;
    MessageString sysMode;
};

struct MessageGpioCommand
{
    MessageGpioCommand():
       preemptive(true)
    {
    }
    MessageHead head;
    qint32 pinNo;
    qint32 command;
    qint32 repeat;
    qint32 interval;
    qint32 postState;
    bool   preemptive;
};

struct MessageGpioEvent
{
    MessageHead head;
    qint32 pinNo;
    qint32 event;
    quint32 keycode;
};

struct MessageGpioRequest
{
    MessageHead head;
    qint32 pinNo;
};

struct MessageGpioResponse
{
    MessageHead head;
    qint32 pinNo;
    qint32 state;
};

struct MessageGetCameraTypeResp
{
    MessageHead head;
    MessageString cameraType;
};

struct MessageLaneDeparture
{
    enum LaneDepartureType{
        NoDeparture = 0,
        LeftDeparture,
        RightDeparture
    };
    MessageHead head;
    LaneDepartureType type;
};

struct MessageProgress{
     MessageHead head;
     qint32 progress;
};

struct MessageDeviceWarning{
    MessageHead head;
    qint32 errCode;
    qint32 errType;
    qint32 errCodeList[128];
    bool functionValid[10];
};

struct MessageUpdateWarning{
    MessageHead head;
    qint32 errorCode;
};

struct MessageBeepType{
    MessageHead head;
    MessageString type;
    quint16 volume;
    bool longPressed;
};

struct MessageCameraData{
    MessageHead head;
    float focus;
    float baseline;
    float pixelSize;
    int opticalCenterX;
    int opticalCenterY;
};

struct MessageVelhicleEvent{
    enum EventType {
        NoEvent = 0,
        RightTurn = 1,
        LeftTurn = 1 << 1,
        TurnEvent = RightTurn | LeftTurn,
        Wiper = 1 << 2,
        Brake = 1 << 3,
        YawRate = 1 << 4,
        Gear = 1 << 5
    };
    MessageHead head;
    quint32 eventType;
    bool isOn;
    float dataFloat;
    int dataInt;
};

struct MessageScreenSystemInfo{
    MessageHead head;
    MessageString sn;
    MessageString version;
    MessageString type;
};

struct MessageSerialNumResp{
    MessageHead head;
    MessageString sn;
};

struct MessageSignalTestEvent
{
    MessageHead head;
    bool isLeftTurnNormal;
    bool isRightTurnNormal;
};

struct MessageCanProtocolTestEvent
{
    MessageHead head;
    bool isCanProtocolNormal;
};

struct MessageOutdoorCalibEvent
{
     MessageHead head;
     bool isOutdoorCalibFinished;
};

struct MessageImuCalibEvent
{
    MessageHead head;
    bool isImuCalibFinished;
};

struct MessageSwitchMode
{
    MessageHead head;
    MessageString mode;
};

struct MessageExtendObstacleInfo
{
    MessageHead head;
    quint8 CIPV;
    quint8 fcwLevel;
    quint8 hmwStatus;
    quint8 reserved;
    float ttc;
    float hmw;
    float relativeSpeed;
    quint32 extDataSize;
    char extData[0];
};

struct MessageExtendLaneInfo
{
    MessageHead head;
    quint16 leftLaneWarningStatus;
    quint16 rightLaneWarningStatus;
    quint32 extDataSize;
    char extData[0];
};

struct MessageCanProtocolSelected
{
    MessageHead head;
    MessageString canProtocol;
};

struct MessageCanProtocolContent
{
    MessageHead head;
    MessageString protocolContent;
};

struct MessagePrepare4Calib
{
    enum CalibType{
        Calib = 0,
        DepthCalib,
        CalibCheck,
        OutdoorDepth,
        AutoCalib,
        DistanceMessure
    };
    MessageHead head;
    int type;
};

struct MessageCanConfiguration
{
    MessageHead head;
    qint16 enable;
    qint32 baudrate;
};

struct MessageFrameRate
{
    MessageHead head;
    MessageString frameRate;
};

struct MessageSensorOffsetSetReq
{
    MessageHead head;
    qint16 leftOffset;
    qint16 rightOffset;
};

struct MessageSensorOffsetSetResp
{
    MessageHead head;
    bool isSetOffsetSucceed;
};

struct MessageFileTransferResp
{
    MessageHead head;
    MessageString fileName;
    bool succeed;
};

#pragma pack(pop)

#endif //SMARTMESSAGE_LIBRARY

#endif // MESSAGEDEF
