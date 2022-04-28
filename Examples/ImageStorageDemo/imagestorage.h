#ifndef IMAGESTORAGE_H
#define IMAGESTORAGE_H

#include <QWidget>
#include <QThread>
#include <QObject>
#include <QRunnable>
#include <QDir>
#include <QMutex>
#include <QQueue>
#include "camerahandler.h"


class FrameSelector;
class ImageStoreThread;
struct RawImageFrame;

class ImageStorage : public QObject, public CameraHandler
{
    Q_OBJECT
public:
    ImageStorage();
    ~ImageStorage() override;

    void init();

    virtual void initCapturePath(bool withDisparity);
    virtual void handleRawFrame(const RawImageFrame *rawFrame);
    void enqueueOneImage(const QByteArray &imageData);
    QByteArray takeOneImage();
    QString getSaveDir();
    void createFrameSelectors();
    void checkAndCreateFolder(const QString &path);
    QString distributeToFolder(const QString &dir);
    void copyUserFiles();
public slots:
    void reset();
    void onCaptureStarted();
    void onCaptureStopped();
signals:
    void captureStarted();
    void captureStopped();

private :
    ImageStoreThread * mImageSaveWorker;
    QString mClockTime;
    QString mPathPre;
    QString mUserFilePath;
    QString mCaptureImagePath;
    mutable QMutex mImageListMutex;
    QQueue<QByteArray> mImageQueue;

    FrameSelector *mCurrentSelector;
    FrameSelector *mRejectSelector;
    FrameSelector *mRemapCaptureWithDisparitySelector;
};


class ImageStoreThread : public QRunnable
{
public:
    ImageStoreThread(ImageStorage *owner);
    void run();        //线程入口函数（工作线程的主函数）
    void stopLater() { mIsWaitingToStop = true; }
	
protected:
    void saveOneImage(const RawImageFrame *rawImage, const char *extData, size_t extSize);
    QString distributeByFrame(const quint16 frameIds);
    QString getImagePrefix(const quint16 frameIds);
    QString getDisparityBitNum(const quint16 format);
    QString getSingleImageName(const QString &prefix, const QString &bitNum,
                               const qint64 time, const quint32 speed, const quint32 width);
							   
private:
    ImageStorage * mOwner;
    bool mIsWaitingToStop;
    bool mExitFlag;
    QString mPicPath;
    QString mWorkPath;
    int mSequnce;
};

#endif // IMAGESTORAGE_H
