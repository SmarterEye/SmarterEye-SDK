#include <QFile>
#include <QDir>
#include "imagestorage.h"
#include "satpext.h"
#include <QDateTime>
#include "frameid.h"
#include "frameformat.h"
#include <QThreadPool>
#include <QDebug>
#include "frameselector.h"
#include <iostream>


#ifdef Q_OS_LINUX
static const QString kStoreImagePath(QString(getenv("HOME")) + "/ADAS_Images");
#else
static const QString kStoreImagePath("D:/ADAS_Images");
#endif


ImageStorage::ImageStorage():
    mImageSaveWorker(new ImageStoreThread(this)),
    mPathPre(kStoreImagePath)
{
      mImageSaveWorker->setAutoDelete(false);
      createFrameSelectors();
}

ImageStorage::~ImageStorage()
{
    QThreadPool::globalInstance()->tryTake(mImageSaveWorker);
    delete mImageSaveWorker;
    mImageSaveWorker = nullptr;
    //frameselector
    delete mRejectSelector;
    mRejectSelector = nullptr;
    delete mRemapCaptureWithDisparitySelector;
    mRemapCaptureWithDisparitySelector = nullptr;
}

void ImageStorage::init()
{
    connect(this,SIGNAL(captureStarted()),this,SLOT(onCaptureStarted()));
    connect(this,SIGNAL(captureStopped()),this,SLOT(onCaptureStopped()));
}

void ImageStorage::copyUserFiles()
{
    static const QVector<QString> kUserFileList = {
        "calibData.yml",
        "depthData.yml",
        "depthRefine.yml",
        "ldw.yml",
        "cameraInstallParam.yml",
        "deviceInfo.json",
        "cameraInfo.json"
    };

    for (auto file : kUserFileList) {
        if (QFile::exists(file)) {
            QFile::copy(file, mUserFilePath + "/" + file);
        }
    }
}

void ImageStorage::initCapturePath(bool withDisparity)
{
    if (mClockTime == "") {
        QDateTime currentTime = QDateTime::currentDateTime();
        mClockTime = currentTime.toString("yyyy_MM_dd");
    }
    const QString &dateDir = mPathPre + "/capture/" + mClockTime;
    checkAndCreateFolder(dateDir);          //create folder
    mCaptureImagePath = distributeToFolder(dateDir);
    QStringList keys;
    keys << "left_images"<< "right_images";
    if(withDisparity){
        keys << "disparity_images";
    }
    if (withDisparity) {
        checkAndCreateFolder(mCaptureImagePath + "/disparity_images");
    }
    checkAndCreateFolder(mCaptureImagePath + "/left_images");
}

void ImageStorage::handleRawFrame(const RawImageFrame *rawFrame)
{
    if (!mCurrentSelector->selectFrame(rawFrame)) return;
    if (mCurrentSelector->allFrameBuffered()) {
        QList<QByteArray> images = mCurrentSelector->getAllFrames();
        while (!images.isEmpty()){
            enqueueOneImage(images.takeFirst());
        }
    }
}

void ImageStorage::enqueueOneImage(const QByteArray &imageData)
{
    QMutexLocker locker(&mImageListMutex);
    mImageQueue.enqueue(imageData);

}

QByteArray ImageStorage::takeOneImage()
{
    QMutexLocker locker(&mImageListMutex);
    return mImageQueue.empty() ? QByteArray() : mImageQueue.dequeue();
}

QString ImageStorage::getSaveDir()
{
    return mCaptureImagePath;
}

void ImageStorage::createFrameSelectors()
{
    mRejectSelector = new RejectSelector();
    mRemapCaptureWithDisparitySelector = new FrameSelector(FrameId::CalibLeftCamera | FrameId::Disparity);
    mCurrentSelector = mRejectSelector;
}

void ImageStorage::checkAndCreateFolder(const QString &path)
{
    QDir saveDir(path);
    if (!saveDir.exists()) {
        saveDir.mkpath(path);
    }
}

void ImageStorage::reset()
{
    mClockTime.clear();
}

void ImageStorage::onCaptureStarted()
{
    mCurrentSelector->reset();
    initCapturePath(true);
    mCurrentSelector = mRemapCaptureWithDisparitySelector;
    bool ret = QThreadPool::globalInstance()->tryStart(mImageSaveWorker);
    if (!ret) {
        qWarning() << "start ImageStoreThread in QThreadPool failed";
    }
}

void ImageStorage::onCaptureStopped()
{
     mCurrentSelector = mRejectSelector;
    // wait for backlog empty
    mImageSaveWorker->stopLater();
    qDebug() << "after click capture stopped, the backlog size is" << mImageQueue.size() << "...";
}

QString ImageStorage::distributeToFolder(const QString &dir)
{
    int index = 1;
    QString qIndex = "%1";
    QString strIndex;

    QString saveDir = dir + "/images_001";
    QDir recordingDir(saveDir);

    while (recordingDir.exists()) {
        recordingDir.setFilter(QDir::Files | QDir::Dirs | QDir::Hidden |
                               QDir::NoSymLinks | QDir::NoDotAndDotDot);
        index++;
        strIndex = qIndex.arg(index, 3, 10, QChar('0'));
        saveDir = dir + "/images" + "_" + strIndex;
        recordingDir = QDir(saveDir);
    }
    return saveDir;
}

/********************************************************/

ImageStoreThread::ImageStoreThread(ImageStorage *owner):
    mOwner(owner),
     mIsWaitingToStop(false)
{
   Q_ASSERT(mOwner);
}

void ImageStoreThread::run()
{
    while(true)
    {
        QByteArray imageBuffer;
        mOwner->takeOneImage().swap(imageBuffer);
        if (!imageBuffer.isEmpty()) {
            auto rawImage = reinterpret_cast<const RawImageFrame *>(imageBuffer.data());

            int headSize = sizeof(RawImageFrame);
            int imageSize = static_cast<int>(rawImage->dataSize);

            bool withExt = (imageBuffer.size() > headSize + imageSize);
            const char* extPtr = nullptr;
            size_t extSize = 0;
            if (withExt) {
                extPtr = reinterpret_cast<const char*>(rawImage->image + rawImage->dataSize);
                extSize = static_cast<size_t>(imageBuffer.size() - headSize - imageSize);
            }
            saveOneImage(rawImage, extPtr, extSize);
        } else {
            // empty
            if (mIsWaitingToStop) {
                mIsWaitingToStop = false;
                break;
            }
        }
    }
}

void ImageStoreThread::saveOneImage(const RawImageFrame *rawImage, const char *extData, size_t extSize)
{
    if (rawImage == nullptr) return;
    // path
    QString saveDir = mOwner->getSaveDir();
    QString frameDistribute = distributeByFrame(rawImage->frameId);
    QString prefix = getImagePrefix(rawImage->frameId);
    QString bitNum = getDisparityBitNum(rawImage->format);
    QString imageName = getSingleImageName(prefix,bitNum, rawImage->time, rawImage->speed, rawImage->width);
    QString imagePath = saveDir + "/" + frameDistribute + "/" + imageName;    //capture
    //save
    QFile imageFile(imagePath);
    imageFile.open(QIODevice::WriteOnly);
    Q_ASSERT_X(imageFile.isOpen(),
               __FUNCTION__, "Create file error!");
    imageFile.write((const char *)rawImage->image, rawImage->dataSize);
    imageFile.close();
}

QString ImageStoreThread::distributeByFrame(const quint16 frameIds)
{
    QString frameDistribute;
    switch (frameIds) {
    case FrameId::Disparity:
        frameDistribute = "disparity_images";
        break;
    case FrameId::LeftCamera:
    case FrameId::CalibLeftCamera:
    case FrameId::LDownSample:
        frameDistribute = "left_images";
        break;
    case FrameId::CalibRightCamera:
    case FrameId::RightCamera:
        frameDistribute = "right_images";
        break;
    }

    return frameDistribute;
}

QString ImageStoreThread::getImagePrefix(const quint16 frameIds)
{
    QString imageNamePrefix;
    switch (frameIds) {
    case FrameId::Disparity:
        imageNamePrefix = "disparity";
        break;
    case FrameId::CalibLeftCamera:
    case FrameId::LDownSample:
        imageNamePrefix = "LRemap";
        break;
    case FrameId::CalibRightCamera:
        imageNamePrefix = "RRemap";
        break;
    case FrameId::LeftCamera:
        imageNamePrefix = "L";
        break;
    case FrameId::RightCamera:
        imageNamePrefix = "R";
        break;
    }
    return imageNamePrefix;
}

QString ImageStoreThread::getDisparityBitNum(const quint16 format)
{
    QString disparityBitNum;
    switch(format) {
    case FrameFormat::Disparity7:
        disparityBitNum = "_0";
        break;
    case FrameFormat::Disparity8:
        disparityBitNum = "_1";
        break;
    case FrameFormat::Disparity12:
        disparityBitNum = "_4";
        break;
    case FrameFormat::Disparity16:
    case FrameFormat::DisparityDens16:
        disparityBitNum = "_5";
        break;
    case FrameFormat::DisparitySparse:
        disparityBitNum = "_16";
        break;
    }
    return disparityBitNum;
}

QString ImageStoreThread::getSingleImageName(const QString &prefix, const QString &bitNum, const qint64 time, const quint32 speed, const quint32 width)
{
    QString suffix = (prefix == "disparity") ? ".dat" : ".raw";
    return (prefix + bitNum + QString ("_%1").arg(time, 20, 10, QChar('0'))
            + "_W" + QString::number(width)+ "_Speed_" + QString::number(speed) + suffix);
}
