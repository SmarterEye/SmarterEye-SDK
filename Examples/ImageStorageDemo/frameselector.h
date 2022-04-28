#ifndef FRAMESELECTOR_H
#define FRAMESELECTOR_H

#include <QObject>
#include <QMap>
#include <QMutex>
#include <QList>

struct RawImageFrame;

class FrameSelector {
public:
    FrameSelector(quint16 ids = 0);
    virtual ~FrameSelector();
    virtual bool selectFrame(const RawImageFrame *frame);
    virtual void reset();
    bool allFrameBuffered();
    quint16 selectedFrameIds();
    QList<QByteArray> getAllFrames();

private:
    quint16 mSelectedIds;
    int mFrameNum;
    qint64 mBufferedTimeStamp;
    QMap<quint16, QByteArray> mImages;
    QMutex mImageLocker;
};

using RejectSelector = FrameSelector;

#endif // FRAMESELECTOR_H
