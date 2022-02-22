#ifndef CONNECTOR_H
#define CONNECTOR_H
#include "satpext.h"
#include <QObject>
#include "satp_global.h"

class QThread;

namespace SATP {

class Protocol;
class TcpProtocol;

class SATPSHARED_EXPORT Connector : public QObject
{
    Q_OBJECT
public:
    explicit Connector(QObject *parent = nullptr);
    virtual ~Connector();
    void connectTo(const QString &hostName, quint16 port = SaptPort::Default);
    void reconnect();
    void disconnectServer();
    Protocol *getProtocol();
    inline QString getHostName(){return mHostName;}
    bool isConnected();
public slots:

protected:
    void init();
    void createProtocol();
    void timerEvent(QTimerEvent *event);
    void connectServer();

private:
    QString mHostName;
    quint16 mPort;
    int mReconnectTimerId;
    TcpProtocol *mProtocol;
    QThread *mThread;
};

}
#endif // CONNECTOR_H
