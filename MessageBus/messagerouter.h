#ifndef MESSAGEROUTER_H
#define MESSAGEROUTER_H

class MessageSlot;

class MessageRouter
{
public:
    virtual void routeMessage(int type, const char *message, int size, MessageSlot *source) = 0;
};

#endif // MESSAGEROUTER_H
