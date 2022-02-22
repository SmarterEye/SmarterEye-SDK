#include "qasiotcpsocket.h"
#include "qsocketconnection.h"
#include <QMutex>

static QMutex mut;

IOServiceClass * QAsioTcpsocket::ioserver = nullptr;
volatile ulong QAsioTcpsocket::linkCout = 0;

QAsioTcpsocket::QAsioTcpsocket(size_t size,QObject *parent) :
    TcpAbstractSocket(parent),serverBuild(false)
{
    mut.lock();
    if (ioserver == nullptr) {
        ioserver = new IOServiceClass();
    }
    ++ linkCout;
    mut.unlock();
    con_ = new Session(ioserver->getService(),size);
    con_->connection->setQTcp(this);
}

QAsioTcpsocket::QAsioTcpsocket(Session * ses) :
    TcpAbstractSocket(0),serverBuild(true),con_(ses)
{
    int soc = con_->connection->socketDescriptor();
    if (soc != -1) {
        asio::ip::tcp::endpoint peerPoint;
        peerPoint = con_->socket().remote_endpoint();
        TcpAbstractSocket::hostConnected(soc,
                                         QString::fromStdString(peerPoint.address().to_string()),
                                         peerPoint.port());
        setState(ConnectedState);
        setSocketPtr(soc);
    }
    con_->connection->setQTcp(this);
}

QAsioTcpsocket::~QAsioTcpsocket()
{
    con_->connection->setQTcp(nullptr);
    delete con_;
    if (!serverBuild) {
        mut.lock();
        -- linkCout;
        if (linkCout == 0) {
            delete ioserver;
            ioserver = nullptr;
        }
        mut.unlock();
    }
}

void QAsioTcpsocket::do_start()
{
//    if (serverBuild) {
//        con_->connection->do_read();
//    }
}

void QAsioTcpsocket::hostConnected()
{
    setState(ConnectedState);
    emit connected();
    emit stateChange(state());
//    con_->connection->do_read();
}

void QAsioTcpsocket::connectToHost(const QString &hostName, quint16 port)
{
    if (state() != UnconnectedState)
        con_->connection->disconnectFromHost();
    setState(HostFinding);
    con_->connection->connectToHost(hostName,port);
}

void QAsioTcpsocket::disconnectFromHost()
{
    if (state() == UnconnectedState) return;
    con_->connection->disconnectFromHost();
    setState(UnconnectedState);
    emit disconnected();
    emit stateChange(state());
}

void QAsioTcpsocket::sendData(const QByteArray &data)
{
    if (state() == ConnectedState)
        con_->connection->wirteData(data);
}

size_t QAsioTcpsocket::syncSendData(const QByteArray &data)
{
    if (state() == ConnectedState){
        try{
            return con_->connection->syncWirteData(data);
        }catch (const asio::system_error &e) {
            qDebug() << "got error while writing socket:"<< e.what();
            haveErro(e.code().value(), e.what());
            return -1;
        }
    }else{
        return -1;
    }
}

size_t QAsioTcpsocket::syncReadData(char *data, size_t maxLen, bool peek)
{
    if (state() == ConnectedState){
        size_t readBytes = 0;
        while(readBytes < maxLen){
            try {
                size_t ret = con_->connection->syncReadData(data + readBytes, maxLen - readBytes, peek);
                if(ret < 0){
                    return  ret;
                }
                readBytes += ret;
            } catch (asio::system_error e) {
                qDebug() << "got error while reading socket:"<< e.what();
                haveErro(e.code().value(), e.what());
                return -1;
            }
        }
        return readBytes;
    }else{
        return -1;
    }
}

void QAsioTcpsocket::setSocketOption(TcpAbstractSocket::SocketOption option, bool isEnble, uint value)
{
    con_->connection->setSocketOption(option,isEnble,value);
}

std::pair<bool,int> QAsioTcpsocket::getSocketOption(TcpAbstractSocket::SocketOption opention)
{
    return con_->connection->getSocketOption(opention);
}

int QAsioTcpsocket::resizeClientBackThreadSize(int size)
{
    if (serverBuild) return -1;
    return ioserver->setThreadSize(size);
}
