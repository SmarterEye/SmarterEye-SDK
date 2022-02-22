#include "messageslot.h"
#include "messagerouter.h"

void MessageSlot::routeMessage(int type, const char *message, int size)
{
    Q_ASSERT(mMessageRouter != nullptr);
    mMessageRouter->routeMessage(type, message, size, this);
}
