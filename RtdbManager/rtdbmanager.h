#ifndef RTDBMANAGER_H
#define RTDBMANAGER_H

#include <QObject>
#include <QString>
#include <QVector>
#include <qglobal.h>
#include <QSettings>
#include "rtdbmanager_global.h"
#include "service.h"
#include "realtimedatabase.h"

class RealtimeDatabase;
class TripMilesManager;
using RtdbItem = RealtimeDatabase::RtdbItem;

class RTDBMANAGERSHARED_EXPORT RtdbManager : public QObject, public Service
{
    Q_OBJECT
public:
    RtdbManager();
    virtual ~RtdbManager();
    inline RealtimeDatabase *getRtdb(){return mRtdb;}
    void init(QString deviceInfoFile, QString cameraInfoFile);

protected:
    void handleMessage(int type, const char *message, int size);
    void timerEvent(QTimerEvent *event);
    bool save();
    bool initRtdbItems();
    bool readDefaultItems();
    bool restoreItemsFromStorge(QString filePath, bool readAll = false);
    bool restoreItemsFromSettings();
    bool addItemsToRtdb();
    void cleanup();
    static QString offlineImagePath();
    void initProjectSettings();

private:
    QString mDeviceInfoFile;
    QString mCameraInfoFile;
    RealtimeDatabase *mRtdb;
    QVector<RtdbItem *> mRtdbItems;
    int mSavingTimerId;
    bool mIsShutdown;
    QSettings *mProjectSettings;
};

#endif // RTDBMANAGER_H
