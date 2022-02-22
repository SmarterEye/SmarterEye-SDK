#ifndef MESSAGEBUS_H
#define MESSAGEBUS_H

#include "messagebus_global.h"
#include "messagerouter.h"
#include <QVector>

class MessageSlot;

class MESSAGEBUSSHARED_EXPORT MessageBus : public MessageRouter
{
public:
    MessageBus(bool isMaster = false);
    void routeMessage(int type, const char *message, int size, MessageSlot *source);
    void registerService(MessageSlot *service);
    void unregisterService(MessageSlot *service);
    bool isMasterConnected();
private:
    QVector<MessageSlot *> mServices;
    bool mIsMaster;
};

#endif // MESSAGEBUS_H
