#ifndef FIRMWAREUPDATER_H
#define FIRMWAREUPDATER_H

#include <QObject>
#include <QUdpSocket>
#include <QThread>
#include <QMap>
#include "stereocamera_global.h"
#include "blockhandler.h"
#include "filesenderhandler.h"
#include "service.h"
namespace SATP {
    class Protocol;
    class FileSender;
}
class RealtimeDatabase;

class STEREOCAMERASHARED_EXPORT FirmwareUpdater : public QObject,
        public SATP::BlockHandler,
        public SATP::FileSenderHandler,
        public Service
{
    Q_OBJECT
public:
    enum UpdateStatus{
        Idle = 0,
        Uploading,
        Upgrading
    };
    explicit FirmwareUpdater(SATP::Protocol *protocol, RealtimeDatabase *rtdb, QObject *parent = nullptr);
    virtual ~FirmwareUpdater();
    //override BlockHandler
    bool handleReceiveBlock(quint32 dataType, const char *block, int size);
    void handleReady();
    void handleReset();
    //override FileSenderHandler
    void handleSendFileFinished(const QString &fileName);
    void handleUpgradeProgress(int progress);
    int getUpdateStatus();
    int getUpdateWarning(){return mUpdateWarning;}
    void initSocket();
signals:
    void updateEvent(const QString &event);
public slots:
    int update(const QString &path);
    double getUpdateProgress();
    bool isDeviceHighTemperature();
protected:
    void handleUpdateResp(const char *block, int size);
    void triggerEvent(const QString &event);
    void handleMessage(int type, const char *message, int size);
    void timerEvent(QTimerEvent *event);
    void startProgressCheckTimer();
    void decodeUDPdatagram(QString datagram);
    void handleOldWarningMessage(int type, const char *message);
protected slots:
    void processPendingDatagram();
    void onErrorOccur(QAbstractSocket::SocketError sockError);

private:
    SATP::Protocol *mProtocol;
    SATP::FileSender *mFileSender;
    RealtimeDatabase *mRtdb;
    UpdateStatus mUpdateStatus;
    QByteArray mFirmwareMd5;
    int mUpgradeProgress;
    bool mDeviceFailure;
    bool mHighTemperature;
    int mErrorCode;
    int mUpdateWarning;
    int mProgressCheckTimerId;
    QUdpSocket *mUdpSocket;
    QString mProductModel;
};

#endif // FIRMWAREUPDATER_H
