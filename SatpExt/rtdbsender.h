#ifndef RTDBADAPTER_H
#define RTDBADAPTER_H

#include <QObject>
#include "satpext_global.h"
#include "rtdbservice.h"
#include "blockhandler.h"

namespace SATP {
    class Protocol;
}

class SATPEXTSHARED_EXPORT RtdbSender : public RtdbService, public SATP::BlockHandler
{
    Q_OBJECT
public:
    explicit RtdbSender(RealtimeDatabase *rtdb, SATP::Protocol *protocol, QObject *parent = nullptr);
    virtual ~RtdbSender();
    //override BlockHandler
    bool handleReceiveBlock(uint32_t dataType, const char *block, int size);
    void handleReady();

protected:
    void handleMessage(int type, const char *message, int size);
    void sendRtdb();

private:
    SATP::Protocol *mProtocol;
};

#endif // RTDBADAPTER_H
