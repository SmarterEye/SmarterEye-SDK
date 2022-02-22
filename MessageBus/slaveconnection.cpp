#include <QDataStream>
#include <QTimer>
#include "slaveconnection.h"
#include "messagerouter.h"
#include "packet.h"

SlaveConnection::SlaveConnection(QObject *parent) : QObject(parent)
{
    mSocket = new QLocalSocket(this);
    connect(mSocket, &QLocalSocket::connected, this, &SlaveConnection::connectedToServer);
    connect(mSocket, &QLocalSocket::disconnected, this, &SlaveConnection::disconnectedFromServer);
    connect(mSocket, &QLocalSocket::readyRead, this, &SlaveConnection::readyRead);
    connect(mSocket,
            static_cast<void(QLocalSocket::*)(QLocalSocket::LocalSocketError)>(&QLocalSocket::error),
            this,
            &SlaveConnection::onErrorOccured);

    connect(this, &SlaveConnection::sendData, this, &SlaveConnection::sendToSocket);
    mReconnectTimerId = startTimer(2000);
}

void SlaveConnection::timerEvent(QTimerEvent *event)
{
    Q_UNUSED(event)
    if (mSocket->state() == QLocalSocket::ConnectingState ||
        mSocket->state() == QLocalSocket::ConnectedState ) {
        return;
    }
    mSocket->abort();
    mSocket->connectToServer(tr(LOCAL_SERVICE_NAME));
}

void SlaveConnection::connectedToServer()
{
    //say hello!
    messageArrive(0, 0, 0);
}

void SlaveConnection::disconnectedFromServer()
{

}

void SlaveConnection::readyRead()
{
    if (mSocket->bytesAvailable() <= 0) return;

    QByteArray buffer;
    buffer = mSocket->readAll();

    mRecvBuffer.append(buffer);

    int packetLen;
    int totalLen = mRecvBuffer.size();

    while (totalLen) {
        QDataStream in(mRecvBuffer);
        if (totalLen < (int)sizeof(PacketHeader)) break;

        PacketHeader head;
        in >> head;

        if (head.token != PACKET_START_TOKEN) {
            mSocket->close();
            return;
        }
        packetLen = head.length + sizeof(PacketHeader);

        //not enough for complete packet length, skip and wait for more data
        if (totalLen < packetLen) break;

        QByteArray message = mRecvBuffer.mid(sizeof(PacketHeader), head.length);

        routeMessage(head.type, message.data(), head.length);

        buffer = mRecvBuffer.right(totalLen - packetLen);
        totalLen = buffer.size();
        mRecvBuffer = buffer;
    }
}

void SlaveConnection::sendToSocket()
{
    QMutexLocker locker(&mSendBufferMutex);
    mSocket->write(mSendBuffer);
}

void SlaveConnection::messageArrive(int type, const char* message, int size)
{
    if (mSocket->state() != QLocalSocket::ConnectedState) return;

    PacketHeader head;
    head.token = PACKET_START_TOKEN;
    head.length = size;
    head.type = type;
    head.source = 0;
    head.target = 0;
    head.format = 0;

    QByteArray headData;
    QDataStream out(&headData, QIODevice::WriteOnly);
    out << head;

    QByteArray sendBuffer(message, size);
    sendBuffer.prepend(headData);
    {
        QMutexLocker locker(&mSendBufferMutex);
        mSendBuffer = sendBuffer;
    }

    //Asyncally running in main thread.
    emit sendData();
}

bool SlaveConnection::isConnected()
{
    return mSocket->state() == QLocalSocket::ConnectedState;
}

void SlaveConnection::onErrorOccured(QLocalSocket::LocalSocketError socketError)
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

