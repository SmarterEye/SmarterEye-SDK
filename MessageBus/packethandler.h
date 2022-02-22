#ifndef PACKETHANDLER_H
#define PACKETHANDLER_H
#include <QByteArray>

class PacketHandler
{
public:
    virtual void packetArrived(const QByteArray &packet) = 0;
};

#endif // PACKETHANDLER_H
