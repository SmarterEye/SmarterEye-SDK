#include <QCoreApplication>
#include "protocol.h"
#include "blockhandler.h"
#include "connector.h"
#include <iostream>
#include <QDateTime>

using namespace std;

class MyBlockHandler : public SATP::BlockHandler
{
public:
    explicit MyBlockHandler(SATP::Protocol *protocol);
    //override.
    bool handleReceiveBlock(quint32 dataType, const char *block, int size);
    void handleReady();

private:
    SATP::Protocol *mProtocol;
    qint64 mReceived;
    qint64 mFirstReceivedTime;
};

MyBlockHandler::MyBlockHandler(SATP::Protocol *protocol)
    : mProtocol(protocol)
{
    protocol->registerBlockHandler(this);
    mReceived = 0;
}

void MyBlockHandler::handleReady()
{
    mReceived = 0;
}

bool MyBlockHandler::handleReceiveBlock(quint32 dataType, const char *block, int size)
{
    if (mReceived == 0) {
        mFirstReceivedTime = QDateTime::currentSecsSinceEpoch();
    }
    mReceived += size;
    static int count  = 0;
    if (++count % 100 == 0) {
        qint64 timeSpan = QDateTime::currentSecsSinceEpoch() - mFirstReceivedTime;
        cout << "receive bytes: " << mReceived << " , speed: "
             << mReceived / (timeSpan * 1024 * 1024) << " MB/s." << endl;
    }
    return true;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    SATP::Connector *connector = new SATP::Connector();
    connector->connectTo("192.168.100.100");
    SATP::Protocol *protocol = connector->getProtocol();
    MyBlockHandler *handler = new MyBlockHandler(protocol);

    return a.exec();
}
