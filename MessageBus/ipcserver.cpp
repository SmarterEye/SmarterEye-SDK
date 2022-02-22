#include <QThread>
#include "packet.h"
#include "ipcserver.h"
#include "masterconnection.h"


IpcServer::IpcServer(QObject *parent) : QObject(parent)
{

}

IpcServer& IpcServer::instance()
{
    static IpcServer *server = new IpcServer();
    return *server;
}

void IpcServer::init()
{
    mLocalServer.reset(new QLocalServer(this));
    if (!mLocalServer->listen(tr(LOCAL_SERVICE_NAME))) {
        if (!QLocalServer::removeServer(tr(LOCAL_SERVICE_NAME))) {
            Q_ASSERT(false);
        }
        if (!mLocalServer->listen(tr(LOCAL_SERVICE_NAME))) {
            Q_ASSERT(false);
        }
    }
    connect(mLocalServer.data(), &QLocalServer::newConnection, this, &IpcServer::onNewConnectionIncoming);
}

void IpcServer::addPacketHandler(PacketHandler *handler)
{
    mPacketHandlers << handler;
}

void IpcServer::onNewConnectionIncoming()
{
    QLocalSocket *socket = mLocalServer->nextPendingConnection();
    MasterConnection *conn = new MasterConnection(this, socket);
    connect(conn, &MasterConnection::died, this, &IpcServer::onConnectionDied);
    mPacketHandlers << conn;
}

void IpcServer::onConnectionDied(MasterConnection *conn)
{
    mPacketHandlers.removeAll((PacketHandler *)conn);
    delete conn;
}

void IpcServer::routePacket(const QByteArray &packet, PacketHandler *source)
{
    for (PacketHandler *handler : mPacketHandlers) {
        if (handler != source) {
            handler->packetArrived(packet);
        }
    }
}

bool IpcServer::ready()
{
    return mPacketHandlers.size() > 1;
}

