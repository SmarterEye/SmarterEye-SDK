#ifndef MESSAGEDEFINE
#define MESSAGEDEFINE
#include <qglobal.h>


#define LOCAL_SERVICE_NAME   "MessageBusService"
#define PACKET_START_TOKEN   0xFAFBFCFD

#pragma pack(push, 1)

class QDataStream;

struct PacketHeader
{
    quint32 token;
    quint16 length;
    quint16 source;
    quint16 target;
    quint16 format;
    quint16 type;
};

QDataStream& operator << (QDataStream& out, const PacketHeader& head);
QDataStream& operator >> (QDataStream& in, PacketHeader& head);

#pragma pack(pop)
#endif // MESSAGEDEFINE

