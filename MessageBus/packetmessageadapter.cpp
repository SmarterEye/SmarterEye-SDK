#include <QDataStream>
#include "packetmessageadapter.h"
#include "packet.h"
#include "ipcserver.h"
#include "packetrouter.h"

PacketMessageAdapter::PacketMessageAdapter(PacketRouter *router):
    mPacketRouter(router)
{

}

void PacketMessageAdapter::packetArrived(const QByteArray &packet)
{
    QDataStream in(packet);
    PacketHeader head;
    in >> head;
    QByteArray message = packet.mid(sizeof(PacketHeader), head.length);
    routeMessage(head.type, message.data(), head.length);
}

void PacketMessageAdapter::messageArrive(int type, const char* message, int size)
{
    if (!mPacketRouter->ready()) return;

    PacketHeader head;
    head.token = PACKET_START_TOKEN;
    head.length = size;
    head.type = type;
    head.source = 0;
    head.target = 0;
    head.format = 0;

    QByteArray headData;
    QDataStream out(&headData, QIODevice::WriteOnly);
    out << head;

    QByteArray sendBuffer(message, size);
    sendBuffer.prepend(headData);
    mPacketRouter->routePacket(sendBuffer, this);
}
