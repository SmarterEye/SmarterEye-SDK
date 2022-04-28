#include "frameselector.h"
#include "frameid.h"
#include "satpext.h"
#include <QBitArray>
#include <QMutexLocker>
#include <QDebug>
#include <QObject>
#include <QDebug>
FrameSelector::FrameSelector(quint16 ids) :
    mSelectedIds(ids)
{
    mFrameNum = QBitArray::fromBits(reinterpret_cast<const char*>(&mSelectedIds), 16).count(true) ;
}
FrameSelector::~FrameSelector()
{
    reset();
}
void FrameSelector::reset()
{
    QMutexLocker locker(&mImageLocker);
    mImages.clear();
    mBufferedTimeStamp = 0;
}

bool FrameSelector::allFrameBuffered()
{
    QMutexLocker locker(&mImageLocker);
    return mImages.count() == mFrameNum;
}

quint16 FrameSelector::selectedFrameIds()
{
    return mSelectedIds;
}

bool FrameSelector::selectFrame(const RawImageFrame *frame)
{
    if(frame->frameId == 0)return false;
    if(frame->time != mBufferedTimeStamp){
        reset();
    }
    QByteArray image(reinterpret_cast<const char*>(frame), sizeof(RawImageFrame) + frame->dataSize + frame->index);

    {
        QMutexLocker locker(&mImageLocker);
        Q_ASSERT(!mImages.contains(frame->frameId));
        mImages.insert(frame->frameId, image);
    }

    mBufferedTimeStamp = frame->time;
    return true;
}
QList<QByteArray> FrameSelector::getAllFrames()
{
    QList<QByteArray> imgs = {};
    {
        QMutexLocker locker(&mImageLocker);
        mImages.values().swap(imgs);
    }
    return imgs;
}
