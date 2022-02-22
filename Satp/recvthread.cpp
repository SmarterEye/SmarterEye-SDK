#include "recvthread.h"
#include "tcpprotocol.h"
#include "protocolunit.h"
#include "qasiotcpsocket.h"

namespace SATP{

enum RECV_STATE{
    NoData,
    WaitingDataUnit,
};


void RecvThread::run(){
    ProtocolUnitHead protocolUnitHead;
    int retval;
    RECV_STATE state = NoData;
    char *recvPtr = nullptr;
    char *dataBuf = nullptr;
    int leftBytes = 0;

    while (!mExitFlag) {
        switch (state) {
        case NoData:
            retval = mParent->getSocket()->peek((char*)&protocolUnitHead, sizeof(protocolUnitHead));
            if (retval == sizeof(protocolUnitHead)){
                if(protocolUnitHead.token != 0x0628){
                    mParent->getSocket()->syncRead((char*)&protocolUnitHead, 1);
                    continue;
                }
                retval = mParent->getSocket()->syncRead((char*)&protocolUnitHead, sizeof(ProtocolUnitHead));
                if(protocolUnitHead.dataUnitSize == 0){
                    mParent->handleFixedProtocolUnit(protocolUnitHead.type);
                    break;
                }
                recvPtr = dataBuf = new char[protocolUnitHead.dataUnitSize];
                leftBytes = protocolUnitHead.dataUnitSize;
                state = WaitingDataUnit;
            }

            break;
        case WaitingDataUnit:
            retval = mParent->getSocket()->syncRead(recvPtr, leftBytes);
            if(mExitFlag)break;
            if(retval < 0){
                perror("recv()");
                break;
            }
            if(retval < leftBytes){
                recvPtr += retval;
                leftBytes -= retval;
            }else{
                mParent->handleDataUnit(dataBuf, protocolUnitHead.dataUnitSize);
                recvPtr = nullptr;
                dataBuf = nullptr;
                leftBytes = 0;
                state = NoData;
            }
            break;
        default:
            break;
        }

    }
    if(dataBuf)delete [] dataBuf;
    qDebug() << "exit RecvThread";
}

}
