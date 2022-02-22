#ifndef SLAVECONNECTION_H
#define SLAVECONNECTION_H

#include <QObject>
#include <QLocalSocket>
#include <QByteArray>
#include <QMutex>
#include "messageslot.h"

class QLocalSocket;

class SlaveConnection : public QObject, public MessageSlot
{
    Q_OBJECT
public:
    explicit SlaveConnection(QObject *parent = 0);
    void messageArrive(int type, const char* message, int size);
    bool isConnected();

signals:
    void sendData();

protected:
    void timerEvent(QTimerEvent *event);
    void readyRead();
    void onErrorOccured(QLocalSocket::LocalSocketError socketError);
    void connectedToServer();
    void disconnectedFromServer();
    void sendToSocket();

private:
    QLocalSocket* mSocket;
    int mReconnectTimerId;
    QByteArray mRecvBuffer;
    QByteArray mSendBuffer;
    QMutex mSendBufferMutex;
};

#endif // SLAVECONNECTION_H
