#ifndef FILESENDERHANDLER_H
#define FILESENDERHANDLER_H

#include <qglobal.h>

namespace SATP {

class FileSenderHandler
{
public:
    virtual void handleSendFileFinished(const QString &fileName) = 0;
};

}
#endif // FILESENDERHANDLER_H
