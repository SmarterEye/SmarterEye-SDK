#include <QDataStream>
#include <QByteArray>
#include <functional>
#include <QTimer>
#include "masterconnection.h"
#include "packetrouter.h"
#include "packet.h"

MasterConnection::MasterConnection(PacketRouter* router, QLocalSocket* socket, QObject *parent)
    : QObject(parent),
      mPacketRouter(router),
      mSocket(socket)
{
    connect(mSocket, &QLocalSocket::disconnected, this, &MasterConnection::socketDisconnected);
    connect(mSocket, &QLocalSocket::readyRead, this, &MasterConnection::readyRead);
    connect(mSocket,
            static_cast<void(QLocalSocket::*)(QLocalSocket::LocalSocketError)>(&QLocalSocket::error),
            this,
            &MasterConnection::onErrorOccured);

    connect(this, &MasterConnection::sendData, this, &MasterConnection::sendToSocket);
}

void MasterConnection::socketDisconnected()
{
    mSocket->deleteLater();
    //clean resource here.
    emit died(this);
}

void MasterConnection::readyRead()
{
    if (mSocket->bytesAvailable() <= 0) return;

    mRecvBuffer.append(mSocket->readAll());

    qint32 recvBufferLen = mRecvBuffer.size();

    while (recvBufferLen) {
        QDataStream in(mRecvBuffer);
        //not enough for one Packet's head? then skipped and waiting for more data.
        if (recvBufferLen < sizeof(PacketHeader))
        {
            break;
        }

        PacketHeader head;
        in >> head;
        if (head.token != PACKET_START_TOKEN) {
            mSocket->close();
            return;
        }
        qint32 packetLen = head.length + sizeof(PacketHeader);

        //not enough for complete packet length, skip and wait for more datas.
        if (recvBufferLen < packetLen) {
            break;
        }

        QByteArray packet = mRecvBuffer.left(packetLen);

        mPacketRouter->routePacket(packet, this);

        mRecvBuffer = mRecvBuffer.right(recvBufferLen - packetLen);
        recvBufferLen = mRecvBuffer.size();
    }
}

void MasterConnection::sendToSocket()
{
    QMutexLocker locker(&mSendBufferMutex);
    mSocket->write(mSendBuffer);
}

void MasterConnection::packetArrived(const QByteArray &packet)
{
    {
        QMutexLocker locker(&mSendBufferMutex);
        mSendBuffer = packet;
    }
    //Asyncally running in main thread.
    emit sendData();
}

void MasterConnection::onErrorOccured(QLocalSocket::LocalSocketError socketError)
{
    switch (socketError) {
    case QLocalSocket::ServerNotFoundError:
        break;
    case QLocalSocket::ConnectionRefusedError:
        break;
    case QLocalSocket::PeerClosedError:
        break;
    default:
        break;
    }
}

