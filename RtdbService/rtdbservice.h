#ifndef RTDBSERVICE_H
#define RTDBSERVICE_H

#include <QObject>
#include "rtdbservice_global.h"
#include "service.h"
#include "realtimedatabase.h"

class RTDBSERVICESHARED_EXPORT RtdbService : public QObject, public Service
{
    Q_OBJECT
public:
    explicit RtdbService(RealtimeDatabase *rtdb, QObject *parent = nullptr);
    inline RealtimeDatabase *getRtdb(){return mRtdb;}
    inline void setRtdb(RealtimeDatabase *rtdb){mRtdb = rtdb;}
    template <typename ItemT>
    void saveRtdbItem(quint16 key, const ItemT &value)
    {
        mRtdb->putItem(key, value);
        broadcastChange(key);
    }
    void saveRtdbItem(quint16 key, const char *data, quint16 len);

protected:
    virtual void handleMessage(int type, const char *message, int size);
    virtual void handleRtdbChanged(quint16 key){}

    void broadcastChange(quint16 key);

private:
    RealtimeDatabase *mRtdb;
};

#endif // RTDBSERVICE_H
