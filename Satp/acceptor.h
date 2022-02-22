#ifndef ACCEPTOR_H
#define ACCEPTOR_H
#include <QObject>
#include "satp_global.h"

class QAsioTcpServer;
class QAsioTcpsocket;

namespace SATP {

class Protocol;
class TcpProtocol;

class SATPSHARED_EXPORT Acceptor : public QObject
{
public:
    explicit Acceptor(QObject *parent = nullptr);
    void start();
    Protocol *getProtocol();
    static Acceptor &instance();

protected:
    void handleNewConnection(QAsioTcpsocket *sock);

private:
    TcpProtocol *mProtocol;
    QAsioTcpServer *mTcpServer;
};

}
#endif // ACCEPTOR_H
