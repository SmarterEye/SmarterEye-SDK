#ifndef PROTOCOLENHANCED_H
#define PROTOCOLENHANCED_H

#include "protocol.h"
#include <QByteArrayList>

namespace SATP {

class ProtocolEnhanced : public Protocol
{
public:
    virtual void sendBlock(uint32_t dataType, QByteArray &block, TramsmitPriority priority = DropWhenBusy) = 0;
    virtual void sendBlock(uint32_t dataType, QByteArrayList &blocks, TramsmitPriority priority = DropWhenBusy) = 0;
};

}
#endif // PROTOCOLENHANCED_H
