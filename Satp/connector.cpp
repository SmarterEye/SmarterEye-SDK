#include "connector.h"

#include <QThread>
#include <QTimerEvent>
#include <QTimer>
#include "tcpprotocol.h"
#include "qasiotcpsocket.h"

namespace SATP {

static const int kReconnectInvervalMS = 1000;

Connector::Connector(QObject *parent):
    QObject(parent)
{
    init();
}

Connector::~Connector()
{
    if(mThread){
        mThread->exit();
        mThread->wait();
        mThread->deleteLater();
    }
    if(mProtocol){
        mProtocol->deleteLater();
    }
}

Protocol *Connector::getProtocol()
{
    Q_ASSERT(nullptr != mProtocol);
    return mProtocol;
}

void Connector::init()
{
    mThread = new QThread();
    mProtocol = new SATP::TcpProtocol(false);
    mProtocol->setSocket(new QAsioTcpsocket);
    mProtocol->moveToThread(mThread);
    mThread->start();
    moveToThread(mThread);
    mReconnectTimerId = -1;
}

void Connector::timerEvent(QTimerEvent *event)
{
    if (event->timerId() == mReconnectTimerId) {
        connectServer();
    }
}

bool Connector::isConnected()
{
    return mProtocol->isConnected();
}

void Connector::reconnect()
{
    Q_ASSERT(nullptr != mProtocol);
    QTimer::singleShot(0, this, [this](){
        if (mReconnectTimerId < 0) {
            mReconnectTimerId = startTimer(kReconnectInvervalMS);
        }
        QAsioTcpsocket *socket = mProtocol->getSocket();
        socket->disconnectFromHost();
    });
}

void Connector::disconnectServer()
{
    QTimer::singleShot(0, this, [this](){
        if(mReconnectTimerId > 0){
            killTimer(mReconnectTimerId);
            mReconnectTimerId = -1;
        }
        mProtocol->getSocket()->disconnectFromHost();
    });
}

void Connector::connectTo(const QString &hostName, quint16 port)
{
    QString oldHost = mHostName;
    mHostName = hostName;
    mPort = port;

    if (mReconnectTimerId < 0) {
        QTimer::singleShot(0, this, [this](){
            mReconnectTimerId = startTimer(kReconnectInvervalMS);
        });
        connectServer();
    }else{
        reconnect();
    }

}

void Connector::connectServer()
{
    Q_ASSERT(nullptr != mProtocol);

    QAsioTcpsocket *socket = mProtocol->getSocket();

    if (socket->state() == TcpAbstractSocket::ConnectedState
        || socket->state() == TcpAbstractSocket::ConnectingState ) {
        return;
    }

    if (!mHostName.isEmpty()) {
        socket->disconnectFromHost();
        socket->connectToHost(mHostName, mPort);
    }
}

}
