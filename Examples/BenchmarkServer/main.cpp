#include <QCoreApplication>
#include <QThread>

#include "protocolenhanced.h"
#include "acceptor.h"
#include "satpext.h"

static const int kDataUnitTypeBenchmark = 999;
static const int kBufferSize = 1024*900;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //network
    SATP::Acceptor::instance().start();
    SATP::ProtocolEnhanced *protocol =
            dynamic_cast<SATP::ProtocolEnhanced *>(SATP::Acceptor::instance().getProtocol());
    while (true) {
        if(!protocol->isConnected()) {
            QThread::sleep(1);
            continue;
        }

        static char *buffer = new char[kBufferSize];
        QByteArrayList imagePackage;
        imagePackage.append(QByteArray(buffer, kBufferSize));
        protocol->sendBlock(UniqueKey(kDataUnitTypeBenchmark, 0).longKey, imagePackage, SATP::Protocol::WaitToSend);
    }
    return a.exec();
}
