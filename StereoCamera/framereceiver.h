#ifndef FRAMERECEIVER_H
#define FRAMERECEIVER_H

#include <QObject>
#include <QMutex>
#include <QHash>
#include <QList>
#include "stereocamera_global.h"
#include "blockhandler.h"

namespace SATP {
    class Protocol;
}

struct RawImageFrame;
class FrameHandler;

class STEREOCAMERASHARED_EXPORT FrameReceiver : public QObject, public SATP::BlockHandler
{
    Q_OBJECT
public:
    explicit FrameReceiver(SATP::Protocol *protocol, QObject *parent = nullptr);
    virtual ~FrameReceiver();
    //override BlockHandler
    bool handleReceiveBlock(uint32_t dataType, const char *block, int size);
    void handleReady();
    //
    void requestFrame(FrameHandler *frameHandler, uint32_t frameIds);
    QList<FrameHandler *> getCameraHandlers();

public slots:
    void detachFrameHandler(QObject *frameHandler);
    void requestLeftOriginalFrame(QObject *frameHandler);
    void requestRightOriginalFrame(QObject *frameHandler);
    void requestLeftCalibFrame(QObject *frameHandler);
    void requestRightCalibFrame(QObject *frameHandler);
    void requestDisparityFrame(QObject *frameHandler);
    void requestLaneInfoFrame(QObject *frameHandler);
    void requestLaneFrame(QObject *frameHandler);
    void requestObstacleFrame(QObject *frameHandler);
    void requestDisparityFrameWithMask(QObject *frameHandler);
    void requestLeftAndDisparityFrame(QObject *frameHandler);
    void requestDoubleOriginalFrame(QObject *frameHandler);
    void requestTripleFrame(QObject *frameHandler);
    void requestOriginalAndRemapFrames(QObject *frameHandler);
    void requestLeftDownSampleFrame(QObject *frameHandler);
    void requestLeftAndRightFrame(QObject *frameHandler);
    void enableMaxSendFrameInterval();

protected:
    void timerEvent(QTimerEvent *event);
    void requestImage(bool forced = false);
    void publishToHandlers(const RawImageFrame *rawImage);

    inline bool hasFramehandler() {
        QMutexLocker locker(&mLock);
        return mFrameHandlersWithId.size() > 0;
    }

    //ImageSource.
    void requestFrame(QObject *frameHandler, quint16 frameIds);
    quint16 getRequestedIds();

private:
    QHash<FrameHandler *, quint16> mFrameHandlersWithId;
    QMutex mLock;

    SATP::Protocol *mProtocol;
    int mRequestImageTimerId;
};

#endif // FRAMERECEIVER_H
