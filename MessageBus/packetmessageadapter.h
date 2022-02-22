#ifndef PACKETMESSAGEADAPTER_H
#define PACKETMESSAGEADAPTER_H
#include "messageslot.h"
#include "packethandler.h"

class PacketRouter;

// a shortcut from MessageBus to IpcServer.

class PacketMessageAdapter : public PacketHandler, public MessageSlot
{
public:
    PacketMessageAdapter(PacketRouter *router);
    void packetArrived(const QByteArray &packet);
    void messageArrive(int type, const char* message, int size);

private:
    PacketRouter *mPacketRouter;
};

#endif // PACKETMESSAGEADAPTER_H
