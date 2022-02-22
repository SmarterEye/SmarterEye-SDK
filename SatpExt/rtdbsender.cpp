#include "rtdbsender.h"
#include <QTimer>
#include "message.h"
#include "rtdbkeys.h"
#include "realtimedatabase.h"
#include "protocol.h"
#include "satpext.h"

RtdbSender::RtdbSender(RealtimeDatabase *rtdb, SATP::Protocol *protocol, QObject *parent)
    : RtdbService (rtdb, parent),
      mProtocol(protocol)
{
    protocol->registerBlockHandler(this);
}

RtdbSender::~RtdbSender()
{
    mProtocol->unregisterBlockHandler(this);
}

bool RtdbSender::handleReceiveBlock(uint32_t dataType, const char *block, int size)
{
    Q_UNUSED(block)
    Q_UNUSED(size)
    UniqueKey key(dataType);

    switch (key.lowWord()) {
    case DataUnitTypeExt::RequestRtdb:
        sendRtdb();
        return true;
    case DataUnitTypeExt::ChangeRtdbItem:
    {
        UniqueKey key(dataType);
        quint16 rtdbKey = key.highWord();
        saveRtdbItem(rtdbKey, block, size);
    }
        return true;
    default:
        break;
    }
    return false;
}

void RtdbSender::handleReady()
{
}

void RtdbSender::sendRtdb()
{
    if (!mProtocol->isConnected()) {
        return;
    }

    if (!getRtdb()->isLoaded()) {
        getRtdb()->load();
    }

    if (getRtdb()->size() > 0) {
        mProtocol->sendBlock(UniqueKey(DataUnitTypeExt::Rtdb, 0).longKey,
                             getRtdb()->data(),
                             getRtdb()->size());
    }
}

void RtdbSender::handleMessage(int type, const char *message, int size)
{
    RtdbService::handleMessage(type, message, size);
}

