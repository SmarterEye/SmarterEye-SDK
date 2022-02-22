#ifndef IPCSERVER_H
#define IPCSERVER_H

#include <QObject>
#include <QVector>
#include <QLocalServer>
#include <QScopedPointer>
#include "packetrouter.h"

class QLocalServer;
class PacketHandler;
class MasterConnection;

class IpcServer : public QObject, public PacketRouter
{
    Q_OBJECT
public:
    static IpcServer& instance();
    void routePacket(const QByteArray &packet, PacketHandler *source);
    bool ready();
    void init();
    void addPacketHandler(PacketHandler *handler);

protected:
    explicit IpcServer(QObject *parent = 0);
    void onNewConnectionIncoming();
    void onConnectionDied(MasterConnection *conn);

private:
    QScopedPointer<QLocalServer> mLocalServer;
    QVector<PacketHandler *> mPacketHandlers;
};

#endif // IPCSERVER_H
