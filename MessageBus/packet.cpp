#include <QDataStream>
#include "packet.h"

QDataStream& operator << (QDataStream& out, const PacketHeader& head)
{
    out << head.token << head.length << head.source << head.target << head.format << head.type;
    return out;
}

QDataStream& operator >> (QDataStream& in, PacketHeader& head)
{
    in >> head.token >> head.length >> head.source >> head.target >> head.format >> head.type;
    return in;
}

