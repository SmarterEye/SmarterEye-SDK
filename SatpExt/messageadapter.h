#ifndef MESSAGEADAPTER_H
#define MESSAGEADAPTER_H

#include <QObject>
#include <QSet>
#include "satpext_global.h"
#include "service.h"
#include "blockhandler.h"

namespace SATP {
    class Protocol;
}

class SATPEXTSHARED_EXPORT MessageAdapter : public QObject, public Service, public SATP::BlockHandler
{
    Q_OBJECT
public:
    explicit MessageAdapter(SATP::Protocol *protocol, QObject *parent = nullptr);
    virtual ~MessageAdapter();
    //override BlockHandler
    bool handleReceiveBlock(uint32_t dataType, const char *block, int size);
    void handleReady();
    void registerUrgentMessage(int type);

protected:
    void handleMessage(int type, const char *message, int size);

private:
    SATP::Protocol *mProtocol;
    QSet<int> mUrgentMessages;
};

#endif // MESSAGEADAPTER_H
