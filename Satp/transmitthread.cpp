#include "transmitthread.h"
#include "tcpprotocol.h"
#include <QTcpSocket>
#include "qasiotcpsocket.h"

namespace SATP{

void TransmitThread::run(){
    QAsioTcpsocket *so = mParent->getSocket();

    ProtocolUnit dataBuff;

    while (!mExitFlag) {
        dataBuff = mParent->getOneBuffer();
        if(mExitFlag)break;
        while (dataBuff.size() > 0) {
            QByteArray segment = dataBuff.takeFirst();

            so->syncWrite(segment);
            emit bytesWritten(segment.size());
        }
    }
    qDebug() << "exit TransmitThread";
}

}
