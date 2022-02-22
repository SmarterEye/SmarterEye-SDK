#include "rtdbservice.h"
#include <QTimer>
#include "message.h"

RtdbService::RtdbService(RealtimeDatabase *rtdb, QObject *parent) :
    QObject(parent),
    mRtdb(rtdb)
{

}

void RtdbService::handleMessage(int type, const char *message, int size)
{
    Q_UNUSED(size)

    switch (type) {

    case MessageType::RtdbChanged:
    {
        const MessageRtdbChanged *msgRtdbChanged = (const MessageRtdbChanged *)message;
        handleRtdbChanged(msgRtdbChanged->changedKey);
    }
        break;
    default:
        break;
    }
}

void RtdbService::broadcastChange(quint16 key)
{
    QTimer::singleShot(0, this, [this, key](){
        MessageRtdbChanged msgRtdbChanged;
        msgRtdbChanged.changedKey = key;
        sendMessage(MessageType::RtdbChanged, msgRtdbChanged);
    });
}

void RtdbService::saveRtdbItem(quint16 key, const char *data, quint16 len)
{
    mRtdb->putItemData(key, data, len);
    broadcastChange(key);
}
