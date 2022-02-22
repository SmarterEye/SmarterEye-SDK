#ifndef MESSAGESLOT_H
#define MESSAGESLOT_H

#include "messagebus_global.h"

class MessageRouter;

class MESSAGEBUSSHARED_EXPORT MessageSlot
{
public:
    MessageSlot(){mMessageRouter = nullptr;}
    inline void setMessageRouter(MessageRouter *router) {mMessageRouter = router;}
    virtual void messageArrive(int type, const char* message, int size) = 0;

protected:
    void routeMessage(int type, const char *message, int size);

private:
    MessageRouter *mMessageRouter;
};

#endif // MESSAGESLOT_H
