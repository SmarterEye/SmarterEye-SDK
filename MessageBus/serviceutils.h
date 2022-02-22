#ifndef SERVICEUTILS_H
#define SERVICEUTILS_H

#include "messagebus_global.h"

class Service;
class QObject;

class MESSAGEBUSSHARED_EXPORT ServiceUtils
{
public:
    ServiceUtils();
    static void playAudioByName(Service *service, const QString& fileName);
};

#endif // SERVICEUTILS_H
