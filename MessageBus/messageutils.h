#ifndef MESSAGEUTILS_H
#define MESSAGEUTILS_H

#include "messagebus_global.h"

struct MessageString;
class QString;

class MESSAGEBUSSHARED_EXPORT MessageUtils
{
public:
    MessageUtils();
    static void copyQStringToMString(const QString &qstring, MessageString &mstring);
};

#endif // MESSAGEUTILS_H
