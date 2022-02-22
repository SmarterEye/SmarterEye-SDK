#ifndef SERVICE_H
#define SERVICE_H

#include "messagebus_global.h"

#include "messageslot.h"

class QObject;

class MESSAGEBUSSHARED_EXPORT Service : public MessageSlot
{
public:
    Service();
    virtual ~Service(){}
    Service(const Service&&){}
    void messageArrive(int type, const char* message, int size);

    template <typename MessageT>
    void sendMessage(int type, const MessageT &message)
    {
        sendMessage(type, reinterpret_cast<const char *>(&message), sizeof(MessageT));
    }
    void sendMessage(int type, const char* message, int size);
    void sendMessage(int type);
    void writeLog(QString log, int tag = 0);
    void postMessage(int type, QObject *context);

protected:

    virtual void handleMessage(int type, const char *message, int size) = 0;
};

#if !defined(QT_NO_DEBUG)
#define DEBUG_MESSAGE(AGENT, MSG)   AGENT->writeLog(MSG)
#else
#define DEBUG_MESSAGE(AGENT, MSG)
#endif

#endif // SERVICE_H
