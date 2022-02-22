#ifndef IPCCONNECTION_H
#define IPCCONNECTION_H

#include <QObject>
#include <QLocalSocket>
#include <QByteArray>
#include <QMutex>
#include "packethandler.h"

class PacketRouter;

class MasterConnection : public QObject, public PacketHandler
{
    Q_OBJECT
public:
    explicit MasterConnection(PacketRouter* router, QLocalSocket* socket, QObject *parent = 0);
    void packetArrived(const QByteArray &packet);

signals:
    void died(MasterConnection *conn);
    void sendData();

protected:
    void socketDisconnected();
    void readyRead();
    void onErrorOccured(QLocalSocket::LocalSocketError socketError);
    void sendToSocket();

private:
    QLocalSocket* mSocket;
    PacketRouter* mPacketRouter;
    QByteArray mRecvBuffer;
    QByteArray mSendBuffer;
    QMutex mSendBufferMutex;
};

#endif // IPCCONNECTION_H
