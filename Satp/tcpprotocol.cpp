#include "tcpprotocol.h"

#include <functional>
#include <QTimer>
#include <qcoreevent.h>
#include <QDebug>
#include "qasiotcpsocket.h"
#include <QThreadPool>

#include "protocolunit.h"
#include "dataunit.h"
#include "blockhandler.h"
#include "satpext.h"

static quint16 kSatpStartToken = 0x0628;
static const int kMaxSendingProtocolUnitCount = 5;
static const int kMaxSendingSocketBufferSize = 1000000;

static const int kClientRecvTimeoutS = 10;
static const int kServerSendTimeoutS = 12;
static const int kClientSendTimeoutS = 12;
static const int kServerShutDownTimeoutS = 15;
static const int kClientShutdownTimeoutS = 20;

static const int kTickIntervalMS = 1000;

namespace SATP
{

TcpProtocol::TcpProtocol(bool isServer):
    mIsWaiting(0),
    mIsServer(isServer),
    mSocket(nullptr)
{
    mTickTimerId = startTimer(kTickIntervalMS);
    mTransmitThread = nullptr;
    mRecvThread = nullptr;
    reset();
}

TcpProtocol::~TcpProtocol()
{
    reset();
    if (mSocket) {
        mSocket->disconnectFromHost();
        mSocket->disconnect();
        mSocket->deleteLater();
    }
}


void TcpProtocol::reset()
{
    mRecvTick = 0;
    mSendTick = 0;
    mIsHeartbeating = false;
    {
        QMutexLocker locker(&mSendingListMutex);
        mSendingList.clear();
    }
}

bool TcpProtocol::waitToAppend(TramsmitPriority priority)
{
    if (isConnected()){
        if (isAppendable()) {
            return true;
        } else {
            if (priority == DropWhenBusy) {
                qDebug() << "drop packages caused by buffer full!!!";
                return false;
            } else if (priority == EnqueueForcedly){
                return true;
            } else { //Wait to send.
                qDebug() << "waiting for slot";
                QMutexLocker locker(&mWaitingMutex);
                mIsWaiting++;
                mWaitPushData.wait(&mWaitingMutex);
                mIsWaiting--;
                return true;
            }
        }
    } else {
        return false;
    }
}

void TcpProtocol::wakeupAppender(bool forced)
{
    if (forced) {
        mWaitPushData.wakeAll();
        return;
    }
    if (isAppendable() && mIsWaiting) {
        mWaitPushData.wakeOne();
    }
}

void TcpProtocol::sendBlock(uint32_t dataType, const char *block, int size, TramsmitPriority priority)
{
    if (!waitToAppend(priority)) return;

    QByteArrayList blocks;
    if (size > 0) {
        blocks.append(QByteArray(block, size));
    }
    appendBlocks(dataType, blocks);
}

void TcpProtocol::sendBlock(uint32_t dataType, QByteArray &block, TramsmitPriority priority)
{
    if (!waitToAppend(priority)) return;

    QByteArrayList blocks;
    blocks.append(block);
    appendBlocks(dataType, blocks);
}

void TcpProtocol::sendBlock(uint32_t dataType, QByteArrayList &blocks, TramsmitPriority priority)
{
    if (!waitToAppend(priority)) return;

    appendBlocks(dataType, blocks);
}

void TcpProtocol::appendBlocks(quint32 dataType, QByteArrayList &blocks)
{
    DataUnitHead head(dataType);

    blocks.prepend(QByteArray(head.data(), sizeof(DataUnitHead)));
    appendDataUnit(blocks);
}


void TcpProtocol::appendDataUnit(DataUnit &dataUnit)
{
    quint32 dataUnitSize = 0;

    for (auto block: dataUnit) {
        dataUnitSize += block.size();
    }

    ProtocolUnitHead protocolUnitHead{
        kSatpStartToken,
        dataUnitSize,
        LongProtocolUnitFormat,
        0,
    };

    ProtocolUnit &protocolUnit = dataUnit;
    protocolUnit.prepend(QByteArray((const char *)&protocolUnitHead, sizeof(ProtocolUnitHead)));
    appendLongProtocolUnit(protocolUnit);
}

void TcpProtocol::TcpProtocol::appendLongProtocolUnit(ProtocolUnit &protocolUnit)
{
    {
        QMutexLocker locker(&mSendingListMutex);
        mSendingList << protocolUnit;
        mWaitPullData.wakeAll();
    }
}

void TcpProtocol::appendFixedProtocolUnit(quint16 type)
{
    ProtocolUnitHead protocolUnitHead{
        kSatpStartToken,
        0,
        FixedProtocolUnitFormat,
        type,
    };

    ProtocolUnit protocolUnit;
    protocolUnit.append(QByteArray((const char *)&protocolUnitHead, sizeof(ProtocolUnitHead)));
    {
        QMutexLocker locker(&mSendingListMutex);
        mSendingList.prepend(protocolUnit);
        mWaitPullData.wakeAll();
    }
}

bool TcpProtocol::isConnected()
{
    return nullptr != mSocket && mSocket->state() == TcpAbstractSocket::ConnectedState;
}

bool TcpProtocol::isAppendable()
{
    QMutexLocker locker(&mSendingListMutex);
    return mSendingList.size() < kMaxSendingProtocolUnitCount;
}

bool TcpProtocol::isWriteable()
{
    return isConnected() /*&& mSocket->bytesToWrite() <= kMaxSendingSocketBufferSize*/;
}

void TcpProtocol::registerBlockHandler(BlockHandler *blockHandler)
{
    QMutexLocker locker(&mHandlersMutex);
    if (mBlockHandlers.contains(blockHandler)) return;
    mBlockHandlers << blockHandler;
}

void TcpProtocol::unregisterBlockHandler(BlockHandler *blockHandler)
{
    Q_ASSERT(blockHandler);
    QMutexLocker locker(&mHandlersMutex);
    mBlockHandlers.removeAll(blockHandler);
}


void TcpProtocol::handleFixedProtocolUnit(quint16 type)
{
    qInfo() << "got handleFixedProtocolUnit " << type;
    mRecvTick = 0;
    if (HeartBeatReq == type) {
        appendFixedProtocolUnit(HeartBeatResp);
        qInfo() << "got HeartBeatReq .";
    } else if (HeartBeatResp == type) {
        qInfo() << "got HeartBeatResp .";
        mIsHeartbeating = false;
    } else {
        qInfo() << "got HeartBeatCon .";
    }
}

void TcpProtocol::handleDataUnit(const char *dataUnit, int size)
{
    DataUnitHead *du = (DataUnitHead *)dataUnit;
    Q_ASSERT(du->continued == 0);
    mRecvTick = 0;

    const char *block = du->body();
    int bodySize = DataUnitHead::bodySize(size);

    if (!du->continued) {
        shared_ptr<const char> data_ptr(block, [](const char *p){
            delete (p - sizeof (DataUnitHead));
        });
        pushToBlockHandlers(du->dataType, data_ptr, bodySize);
    }
}

void TcpProtocol::pushToBlockHandlers(quint32 dataType, shared_ptr<const char> &block, int size)
{
    QMutexLocker locker(&mHandlersMutex);
    for (BlockHandler *handler : mBlockHandlers) {
        EnhancedHandler *eh = dynamic_cast<EnhancedHandler*>(handler);
        if(eh){
            eh->handleReceiveBlockEnhanced(dataType, block, size);
        }else {
            handler->handleReceiveBlock(dataType, block.get(), size);
        }
    }
}

void TcpProtocol::timerEvent(QTimerEvent *event)
{
    if (!isConnected()) return;

    if (event->timerId() == mTickTimerId) {
        handleTick();
    }
}

void TcpProtocol::handleTick()
{
    ++mRecvTick;
    ++mSendTick;

    if (mIsServer) {
        if (mRecvTick > kServerShutDownTimeoutS) {
            qInfo() << "Server: kServerShutDownTimeoutS .";
            disconnectSocket();
        }

        if (mSendTick > kServerSendTimeoutS) {
            appendFixedProtocolUnit(HeartBeatCon);
            qInfo() << "Server: kServerSendTimeoutS .";
        }

    } else {
        if (mRecvTick > kClientRecvTimeoutS) {
            if (!mIsHeartbeating) {
                qInfo() << "Client: kClientRecvTimeout .";
                appendFixedProtocolUnit(HeartBeatReq);
                mIsHeartbeating = true;
                return;
            }
        }

        if (mRecvTick > kClientShutdownTimeoutS) {
            qInfo() << "Client: kClientShutdownTimeoutS .";
            disconnectSocket();
            return;
        }

        if (mSendTick > kClientSendTimeoutS) {
            if (!mIsHeartbeating) {
                appendFixedProtocolUnit(HeartBeatCon);
                qInfo() << "Client: kClientSendTimeout .";
            }
        }
    }
}

void TcpProtocol::disconnectSocket()
{
    mSocket->disconnectFromHost();
    qInfo() << "aborted!";
}

void TcpProtocol::setSocket(QAsioTcpsocket *socket, bool connected)
{
    mSocket = socket;
    bindSocket();

    if (connected) {
        onSocketConnected();
    }
}

void TcpProtocol::bindSocket()
{
    connect(mSocket, &TcpAbstractSocket::connected, this, &TcpProtocol::onSocketConnected);
    connect(mSocket, &TcpAbstractSocket::disconnected, this, &TcpProtocol::onSocketDisconnected);
    connect(mSocket, &TcpAbstractSocket::bytesWritten, this, &TcpProtocol::onSocketWritten);
}

void TcpProtocol::onSocketWritten(qint64 bytes)
{
    Q_UNUSED(bytes);
    mSendTick = 0;
}

void TcpProtocol::onSocketErrorOccured(int socketError)
{
}

void TcpProtocol::onSocketConnected()
{
    qInfo() << "onSocketConnected!";
    QMutexLocker locker(&mHandlersMutex);
    for (BlockHandler *handler : mBlockHandlers) {
        handler->handleReady();
    }
    if(mTransmitThread == nullptr){
        mTransmitThread = new TransmitThread(this);
        connect(mTransmitThread, &TransmitThread::bytesWritten, this,
                &TcpProtocol::onSocketWritten);
        if(!QThreadPool::globalInstance()->tryStart(mTransmitThread)){
            delete mTransmitThread;
            mTransmitThread = nullptr;
            disconnectSocket();
            qWarning() << "create the transmit thread error!";
        }
    }
    if(mRecvThread == nullptr){
        mRecvThread = new RecvThread(this);
        if(!QThreadPool::globalInstance()->tryStart(mRecvThread)){
            delete mRecvThread;
            mRecvThread = nullptr;
            disconnectSocket();
            qWarning() << "create the received thread error!";
        }

    }
}

void TcpProtocol::onSocketDisconnected()
{
    reset();
    resetHandlers();
    wakeupAppender(true);
    qInfo() << "onSocketDisconnected!";
    if(mTransmitThread != nullptr){
        disconnect(mTransmitThread, &TransmitThread::bytesWritten, this,
                   &TcpProtocol::onSocketWritten);
        mTransmitThread->exit();
        mWaitPullData.wakeAll();
        mTransmitThread = nullptr;
    }
    if(mRecvThread != nullptr){
        mRecvThread->exit();
        mRecvThread = nullptr;
    }
}

void TcpProtocol::resetHandlers()
{
    QMutexLocker locker(&mHandlersMutex);
    for (BlockHandler *handler : mBlockHandlers) {
        handler->handleReset();
    }
}

ProtocolUnit TcpProtocol::getOneBuffer()
{
    QMutexLocker locker(&mSendingListMutex);
    if (mSendingList.size() > 0) {
        wakeupAppender(true);
        return mSendingList.takeFirst();
    } else {
        mWaitPullData.wait(&mSendingListMutex);
        if (mSendingList.size() > 0) {
            wakeupAppender(true);
            return mSendingList.takeFirst();
        }
    }
    return QByteArrayList();
}

}

