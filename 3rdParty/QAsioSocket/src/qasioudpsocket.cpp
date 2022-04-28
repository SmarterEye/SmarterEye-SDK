#include <QMutex>

#include "qasioudpsocket.h"
#include "qudpsocketconnection.h"

static QMutex mut;

IOServiceClass * QAsioUdpsocket::ioserver = nullptr;
volatile ulong QAsioUdpsocket::linkCout = 0;

QAsioUdpsocket::QAsioUdpsocket(size_t size, uint16_t localPort)
{
    mut.lock();
    if (ioserver == nullptr) {
        ioserver = new IOServiceClass();
        if(ioserver != nullptr)
        {
            printf("[file: %s func: %s linenum: %d] creat io server class\n", __FILE__, __FUNCTION__, __LINE__);
        }
    }
    ++ linkCout;
    mut.unlock();

    pUdpSessionCon_ = new UdpSession(ioserver->getService(), size, localPort);
    if(nullptr == pUdpSessionCon_)
    {
        printf("[file: %s func: %s linenum: %d] creat udp session failed\n", __FILE__, __FUNCTION__, __LINE__);
    }
    else
    {
        printf("[file: %s func: %s linenum: %d] creat udp session success\n", __FILE__, __FUNCTION__, __LINE__);
    }
}

QAsioUdpsocket::~QAsioUdpsocket()
{
    if(nullptr != pUdpSessionCon_)
    {
        delete pUdpSessionCon_;
        pUdpSessionCon_ = nullptr;
    }
    
    mut.lock();
    -- linkCout;
    if (linkCout == 0) {
        if(nullptr != ioserver)
        {
            delete ioserver;
            ioserver = nullptr;
        }
    }
    mut.unlock();
}

void QAsioUdpsocket::disconnectFromHost()
{
    pUdpSessionCon_->connection->disconnectFromHost();
    printf("[file: %s func: %s linenum: %d] disconnect from host\n", __FILE__, __FUNCTION__, __LINE__);
}

void QAsioUdpsocket::setRemotePoint(std::string remoteIp, uint16_t remotePort)
{
    pUdpSessionCon_->connection->setUdpSocketRemotePoint(remoteIp, remotePort);
}
void QAsioUdpsocket::setAsyncReadDataHandleCb(void *ptr)
{
    pUdpSessionCon_->connection->setAsyncReadDataHandleCb(ptr);
}

uint16_t QAsioUdpsocket::getLocalPort()
{
    return pUdpSessionCon_->connection->getLocalPort();
}

void QAsioUdpsocket::sendData(const QByteArray &data)
{
    pUdpSessionCon_->connection->wirteData(data);
    printf("[file: %s func: %s linenum: %d] send data\n", __FILE__, __FUNCTION__, __LINE__);
}

void QAsioUdpsocket::StartAsyncReadData()
{
    pUdpSessionCon_->connection->doAsyncRead();
    printf("[file: %s func: %s linenum: %d] start async read data\n", __FILE__, __FUNCTION__, __LINE__);
}

size_t QAsioUdpsocket::syncSendData(const char *pData, size_t dataSize)
{
    try{
        return pUdpSessionCon_->connection->syncWirteData(pData, dataSize);
    }catch (const asio::system_error &e) {
        qDebug() << "got error while writing socket:"<< e.what();
        return -1;
    }
}

size_t QAsioUdpsocket::syncReadData(char *data, size_t maxLen, bool needLoop)
{
    size_t readBytes = 0;
    while(readBytes < maxLen){
        try {
            size_t ret = pUdpSessionCon_->connection->syncReadData(data + readBytes, maxLen - readBytes);
            if(ret < 0){
                return  ret;
            }
            readBytes += ret;

            if(!needLoop)
            {
                break;
            }
        } catch (asio::system_error e) {
            qDebug() << "got error while reading socket:"<< e.what();
            return -1;
        }
    }
    return readBytes;
}

int QAsioUdpsocket::resizeClientBackThreadSize(int size)
{
    return ioserver->setThreadSize(size);
}
