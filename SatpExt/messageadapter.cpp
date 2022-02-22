#include "messageadapter.h"
#include "protocol.h"
#include "satpext.h"

MessageAdapter::MessageAdapter(SATP::Protocol *protocol, QObject *parent)
    : QObject(parent),
      mProtocol(protocol)
{
    protocol->registerBlockHandler(this);
}

MessageAdapter::~MessageAdapter()
{
    mProtocol->unregisterBlockHandler(this);
}

bool MessageAdapter::handleReceiveBlock(uint32_t dataType, const char *block, int size)
{
    UniqueKey key(dataType);

    switch (key.lowWord()) {
    case DataUnitTypeExt::Message:
        sendMessage(key.highWord(), block, size);
        return true;
    }
    return false;
}

void MessageAdapter::handleReady()
{

}

void MessageAdapter::handleMessage(int type, const char *message, int size)
{
    SATP::Protocol::TramsmitPriority priority = SATP::Protocol::DropWhenBusy;
    if (mUrgentMessages.contains(type)) {
        priority = SATP::Protocol::EnqueueForcedly;
    }
    mProtocol->sendBlock(UniqueKey(DataUnitTypeExt::Message, static_cast<uint16_t>(type)).longKey, message, size, priority);
}

void MessageAdapter::registerUrgentMessage(int type)
{
    mUrgentMessages << type;
}
