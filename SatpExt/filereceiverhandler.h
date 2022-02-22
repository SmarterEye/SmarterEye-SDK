#ifndef FILERECEIVERHANDLER_H
#define FILERECEIVERHANDLER_H

#include <qglobal.h>

namespace SATP {

class FileReceiverHandler
{
public:
    virtual void handleReceiveFile(const QString &fileName) = 0;
};

}
#endif // FILERECEIVERHANDLER_H
