#include <QString>
#include "messageutils.h"
#include "message.h"

MessageUtils::MessageUtils()
{

}

void MessageUtils::copyQStringToMString(const QString &qstring, MessageString &mstring)
{
    int len = qstring.length();
    if (len > kGeneralLogLength) len = kGeneralLogLength;
    memcpy(mstring.data, qstring.data(), len * sizeof(QChar));
    mstring.length = len;
}

