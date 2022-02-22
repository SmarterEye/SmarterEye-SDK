#ifndef TCPPROTOCOL_H
#define TCPPROTOCOL_H

#include <QObject>
#include <QVector>
#include <QByteArray>
#include <QMutex>
#include <QWaitCondition>
#include "satp_global.h"
#include "protocolenhanced.h"
#include "transmitthread.h"
#include "recvthread.h"
#include <memory>

class QByteArray;
class QAsioTcpsocket;

using DataUnit = QByteArrayList;
using ProtocolUnit = QByteArrayList;
using namespace std;
namespace SATP
{

class TcpProtocol : public QObject, public ProtocolEnhanced
{
    Q_OBJECT
public:
    TcpProtocol(bool isServer = true);
    virtual ~TcpProtocol();
    //override Protocol
    void sendBlock(uint32_t dataType, const char *block, int size, TramsmitPriority priority = DropWhenBusy);
    void sendBlock(uint32_t dataType, QByteArray &block, TramsmitPriority priority = DropWhenBusy);
    void sendBlock(uint32_t dataType, QByteArrayList &blocks, TramsmitPriority priority = DropWhenBusy);
    bool isAppendable();
    bool isConnected();
    void registerBlockHandler(BlockHandler *blockHandler);
    void unregisterBlockHandler(BlockHandler *blockHandler);
    //TcpProtocol
    void setSocket(QAsioTcpsocket *socket, bool connected = false);
    inline QAsioTcpsocket *getSocket() {return mSocket;}
    ProtocolUnit getOneBuffer();
    void handleFixedProtocolUnit(quint16 type);
    void handleDataUnit(const char *dataUnit, int size);

signals:
    void transmit();

protected:
    bool isWriteable();
    void reset();
    void timerEvent(QTimerEvent *event);
    void handleTick();
    //receive
    void pushToBlockHandlers(quint32 dataType, shared_ptr<const char> &block, int size);
    void resetHandlers();
    //send
    void appendBlocks(quint32 dataType, QByteArrayList &blocks);
    void appendDataUnit(DataUnit &dataUnit);
    void appendLongProtocolUnit(ProtocolUnit &protocolUnit);
    void appendFixedProtocolUnit(quint16 type);
    //mutex
    bool waitToAppend(TramsmitPriority priority);
    void wakeupAppender(bool forced = false);
    //socket
    void bindSocket();
    void disconnectSocket();
    ////socket slots
    void onSocketWritten(qint64 bytes);
    void onSocketErrorOccured(int socketError);
    void onSocketConnected();
    void onSocketDisconnected();

private:
    QVector<ProtocolUnit> mSendingList;
    QMutex mSendingListMutex;
    QVector<BlockHandler *> mBlockHandlers;
    QMutex mHandlersMutex;

    //sending sync
    int mIsWaiting;
    QMutex mWaitingMutex;
    QWaitCondition mWaitPushData;
    QWaitCondition mWaitPullData;

    //session control.
    int mTickTimerId;
    int mRecvTick;
    int mSendTick;
    bool mIsServer;
    bool mIsHeartbeating;

    //socket
    QAsioTcpsocket *mSocket;
    TransmitThread *mTransmitThread;
    RecvThread *mRecvThread;
};

}
#endif // TCPPROTOCOL_H
