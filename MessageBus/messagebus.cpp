#include "messagebus.h"
#include "slaveconnection.h"
#include "ipcserver.h"
#include "packetmessageadapter.h"


MessageBus::MessageBus(bool isMaster):
    mIsMaster(isMaster)
{
    if (isMaster) {
#if !defined Q_OS_ANDROID && !defined Q_OS_IOS
        IpcServer::instance().init();
#endif
        PacketMessageAdapter *adapter = new PacketMessageAdapter(&IpcServer::instance());
        IpcServer::instance().addPacketHandler(adapter);
        adapter->setMessageRouter(this);
        mServices << adapter;
    } else {
        SlaveConnection *conn = new SlaveConnection();
        conn->setMessageRouter(this);
        mServices << conn;
    }
}

bool MessageBus::isMasterConnected()
{
    if (mIsMaster) {
        return true;
    } else {
        SlaveConnection *conn =  (SlaveConnection *)mServices[0];
        return conn->isConnected();
    }
}

void MessageBus::registerService(MessageSlot *service)
{
    Q_ASSERT(service);
    Q_ASSERT(!mServices.contains(service));
    mServices << service;
    service->setMessageRouter(this);
}

void MessageBus::unregisterService(MessageSlot *service)
{
    Q_ASSERT(service);
    mServices.removeAll(service);
}

void MessageBus::routeMessage(int type, const char *message, int size, MessageSlot *source)
{
    for (auto service : mServices) {
        if (service && service != source) {
            service->messageArrive(type, message, size);
        }
    }
}

