#ifndef PACKETROUTER_H
#define PACKETROUTER_H
#include <QByteArray>

class PacketHandler;

class PacketRouter
{
public:
    virtual void routePacket(const QByteArray &packet, PacketHandler *source) = 0;
    virtual bool ready() = 0;
};

#endif // PACKETROUTER_H
