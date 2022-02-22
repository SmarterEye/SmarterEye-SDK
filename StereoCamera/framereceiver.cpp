#include "framereceiver.h"
#include <QTimerEvent>
#include <QTimer>

#include "protocol.h"
#include "satpext.h"
#include "framehandler.h"
#include "frameid.h"

static const int kRequestImageInvervalMS = 3000;

FrameReceiver::FrameReceiver(SATP::Protocol *protocol, QObject *parent):
    QObject(parent),
    mProtocol(protocol)
{
    Q_ASSERT(protocol);
    protocol->registerBlockHandler(this);
    mRequestImageTimerId = startTimer(kRequestImageInvervalMS);
}

FrameReceiver::~FrameReceiver()
{
    mProtocol->unregisterBlockHandler(this);
}

bool FrameReceiver::handleReceiveBlock(uint32_t dataType, const char *block, int size)
{
    UniqueKey key(dataType);

    switch (key.lowWord()) {
    case DataUnitTypeExt::RawImageFrame:
    {
        RawImageFrame *frame = reinterpret_cast<RawImageFrame *>((char*)block);
        int32_t extDataSize = size - static_cast<int>(frame->dataSize + sizeof (RawImageFrame));
        if(extDataSize > 0){
            frame->index = extDataSize;
        }
        publishToHandlers(frame);//
        return true;
    }
    }
    return false;
}

void FrameReceiver::handleReady()
{

}

void FrameReceiver::timerEvent(QTimerEvent *event)
{
    if (event->timerId() == mRequestImageTimerId){
        requestImage();
    }
}

void FrameReceiver::requestImage(bool forced)
{
    if (forced || hasFramehandler()){
        mProtocol->sendBlock(UniqueKey(DataUnitTypeExt::RequestImage, getRequestedIds()).longKey, nullptr, 0);
    }
}

void FrameReceiver::detachFrameHandler(QObject *frameHandler)
{
    FrameHandler *handler = dynamic_cast<FrameHandler *>(frameHandler);
    requestFrame(handler, 0);
}


void FrameReceiver::requestFrame(QObject *frameHandler, quint16 frameIds)
{
    FrameHandler *handler = dynamic_cast<FrameHandler *>(frameHandler);
    requestFrame(handler, frameIds);
}

void FrameReceiver::requestFrame(FrameHandler *frameHandler, uint32_t frameIds) {
    QMutexLocker locker(&mLock);
    if (0 == frameIds) {
        mFrameHandlersWithId.remove(frameHandler);
    } else { 
        if (frameIds==FrameId::DepthRGB) {
        frameIds=FrameId::Disparity;
    }
        mFrameHandlersWithId[frameHandler] = static_cast<quint16>(frameIds);
    }
    QTimer::singleShot(0, this, [this](){
        requestImage();
    });
}

QList<FrameHandler *> FrameReceiver::getCameraHandlers()
{
    QMutexLocker locker(&mLock);
    return mFrameHandlersWithId.keys();
}

quint16 FrameReceiver::getRequestedIds() {
    QMutexLocker locker(&mLock);
    quint16 ids = 0;
    for (auto id : mFrameHandlersWithId.values()) {
        ids |= id;
    }
    return ids;
}

void FrameReceiver::requestLeftOriginalFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::LeftCamera);
}

void FrameReceiver::requestRightOriginalFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::RightCamera);
}

void FrameReceiver::requestLeftCalibFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::CalibLeftCamera);
}

void FrameReceiver::requestRightCalibFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::CalibRightCamera);
}

void FrameReceiver::requestDisparityFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::Disparity);
}

void FrameReceiver::requestLaneInfoFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::Lane | FrameId::LaneExt);
}

void FrameReceiver::requestLaneFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::Lane);
}

void FrameReceiver::requestObstacleFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::Obstacle);
}

void FrameReceiver::requestDisparityFrameWithMask(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::Compound);
}

void FrameReceiver::requestLeftAndDisparityFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::CalibLeftCamera
                            |FrameId::Disparity);
}

void FrameReceiver::requestDoubleOriginalFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::LeftCamera
                              |FrameId::RightCamera);
}

void FrameReceiver::requestTripleFrame(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::CalibLeftCamera
                              |FrameId::CalibRightCamera
                              |FrameId::Disparity );
}

void FrameReceiver::requestOriginalAndRemapFrames(QObject *frameHandler)
{
    requestFrame(frameHandler, FrameId::LeftCamera | FrameId::RightCamera
                 | FrameId::CalibLeftCamera| FrameId::CalibRightCamera);
}

void FrameReceiver::requestLeftDownSampleFrame(QObject *frameHandler){
    requestFrame(frameHandler, FrameId::LDownSample);
}

void FrameReceiver::requestLeftAndRightFrame(QObject *frameHandler){
    requestFrame(frameHandler, FrameId::CalibLeftCamera
                 |FrameId::CalibRightCamera);
}

void FrameReceiver::enableMaxSendFrameInterval()
{
    QTimer::singleShot(0, this, [this](){
        mProtocol->sendBlock(UniqueKey(DataUnitTypeExt::EnableMaxSendFrameInterval, 0).longKey, nullptr, 0);
    });
}

/////////
void FrameReceiver::publishToHandlers(const RawImageFrame *rawImage)
{
    QMutexLocker locker(&mLock);
    for (auto pair = mFrameHandlersWithId.begin(); pair != mFrameHandlersWithId.end(); ++pair) {
        FrameHandler *handler = pair.key();
        quint16 ids = pair.value();

        if (rawImage->frameId & ids) {
            handler->handleRawFrame(rawImage);
        }
    }
}
