#ifndef RTDBRECEIVER_H
#define RTDBRECEIVER_H

#include <QObject>
#include <QVariant>
#include <QHash>
#include "stereocamera_global.h"
#include "service.h"
#include "blockhandler.h"

namespace SATP {
    class Protocol;
}

class RealtimeDatabase;

class STEREOCAMERASHARED_EXPORT RtdbReceiver : public QObject, public Service, public SATP::BlockHandler
{
    Q_OBJECT
public:
    explicit RtdbReceiver(SATP::Protocol *protocol, QObject *parent = nullptr);
    virtual ~RtdbReceiver();
    //override BlockHandler
    bool handleReceiveBlock(uint32_t dataType, const char *block, int size);
    void handleReady();

    inline RealtimeDatabase *getRtdb(){return mRtdb;}

signals:
    void rtdbLoaded();

public slots:
    void request();
    void clear();
    bool isLoaded();
    //Realtime Database extension.
    QList<int> getRtdbKeys();
    quint16 getKeyByName(const QString &name);
    QString getRtdbItemName(int key);
    QVariant getRtdbItemValue(int key);
    bool getRtdbItemPersistent(int key);
    QVariant getRtdbValueByName(const QString &name);
    void saveRtdbItem(int key, QVariant value);
    void saveRtdbItemByName(const QString &name, QVariant value);

protected:
    void handleMessage(int type, const char *message, int size);
    void handleRtdb(const char *block, int size);

private:
    SATP::Protocol *mProtocol;
    RealtimeDatabase *mRtdb;
    QHash <QString, quint16> mNameKeyMap;
    bool mNeedClear;
};

#endif // RTDBRECEIVER_H
