#include "acceptor.h"
#include <QMutex>
#include <QWaitCondition>
#include <QThread>
#include <QTcpSocket>
#include <QTimerEvent>
#include <QTimer>
#include "qasiotcpserver.h"
#include "qasiotcpsocket.h"
#include "tcpprotocol.h"
#include "satpext.h"

namespace SATP {

class AcceptorThread : public QThread
{
public:
    AcceptorThread(QObject *parent = 0):
        QThread(parent)
    {

    }

    void run()
    {
        mAcceptor = new Acceptor();
        mWaitCondition.wakeOne();
        exec();
    }

    void waitToCreate()
    {
        start();
        QMutexLocker locker(&mWaitingMutex);
        mWaitCondition.wait(&mWaitingMutex);
    }
    Acceptor *mAcceptor;
    QMutex mWaitingMutex;
    QWaitCondition mWaitCondition;
};

Acceptor::Acceptor(QObject *parent):
    QObject(parent)
{
    mTcpServer = new QAsioTcpServer();
    connect(mTcpServer, &QAsioTcpServer::newConnection, this, &Acceptor::handleNewConnection);
    mProtocol = new TcpProtocol();
}

void Acceptor::start()
{
    QTimer::singleShot(0, mTcpServer, [this](){
        mTcpServer->listen(SaptPort::Default);
    });
}

Protocol *Acceptor::getProtocol()
{
    return mProtocol;
}

Acceptor &Acceptor::instance()
{
    static AcceptorThread *acceptorThread = nullptr;
    if (nullptr == acceptorThread) {
        acceptorThread = new AcceptorThread();
        acceptorThread->waitToCreate();
    }
    return *acceptorThread->mAcceptor;
}

void Acceptor::handleNewConnection(QAsioTcpsocket *newSocket)
{
    QAsioTcpsocket *oldSocket = mProtocol->getSocket();

    if (nullptr != oldSocket && (oldSocket->state() == TcpAbstractSocket::ConnectedState
        || oldSocket->state() == TcpAbstractSocket::ConnectingState )) {
        //reject new connection.
        qInfo() << "reject new connection";
        newSocket->disconnectFromHost();
        delete newSocket;
        return;
    }

    qInfo() << newSocket->peerIp() << " connected!";
    if (nullptr != oldSocket) {
        delete oldSocket;
    }

    mProtocol->setSocket(newSocket, true);
//    newSocket->do_start();
}

}
