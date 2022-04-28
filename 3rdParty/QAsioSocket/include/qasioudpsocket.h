#ifndef QASIOUDPSOCKET_H
#define QASIOUDPSOCKET_H

#include "qasiosocket.h"

struct UdpSession;
class IOServiceClass;

class QASIOSOCKET_EXPORT QAsioUdpsocket : public QObject
{
public:
    explicit QAsioUdpsocket(size_t size = 4096, uint16_t localPort = 0);
    ~QAsioUdpsocket();

signals:

public slots:
    void disconnectFromHost();
    int resizeClientBackThreadSize(int size);

public:
    void sendData(const QByteArray &data);
    size_t syncSendData(const char *pData, size_t dataSize);
    size_t syncReadData(char *data, size_t maxLen, bool needLoop);
    void StartAsyncReadData();

    void setRemotePoint(std::string remoteIp, uint16_t remotePort);
    void setAsyncReadDataHandleCb(void *ptr);

    uint16_t getLocalPort();

private:
    bool serverBuild;
    UdpSession *pUdpSessionCon_;
    static IOServiceClass * ioserver;
    static volatile ulong linkCout;
    Q_DISABLE_COPY(QAsioUdpsocket)
};

#endif
