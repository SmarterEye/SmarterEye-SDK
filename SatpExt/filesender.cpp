#include "filesender.h"

#include <QThreadPool>
#include <QFile>
#include <QFileInfo>
#include <QDebug>
#include "protocol.h"
#include "filesenderhandler.h"
#include "satpext.h"
#include "dataunit.h"

namespace SATP
{

FileSender::FileSender(Protocol *protocol, FileSenderHandler *senderHandler):
    mProtocol(protocol),
    mSenderHandler(senderHandler)
{
    protocol->registerBlockHandler(this);
    mIsRunning = false;
    mIsCanceling = false;
}

FileSender::~FileSender()
{
    mProtocol->unregisterBlockHandler(this);
}

void FileSender::run()
{
    mIsRunning = true;
    mIsCanceling = false;
    sendFileSynchronously();
    mIsCanceling = false;
    mIsRunning = false;
}

void FileSender::sendFileSynchronously()
{
    QFile file(mFilePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qInfo() << "sendFileSynchronously(), failed to open file!";
        return;
    }

    mFileSize = file.size();
    mSizeToSend = mFileSize;
    if (mSizeToSend <= 0) {
        qInfo() << "sendFileSynchronously(), empty file!";
        return;
    }

    sendFileHeader(mSizeToSend);

    while (mSizeToSend > 0) {

        if (mIsCanceling) break;

        QByteArray block;
        block = file.read(maxBlockSize());
        mSizeToSend -= block.size();
        mProtocol->sendBlock(DataUnitTypeExt::FileData, block.data(), block.size(), Protocol::WaitToSend);
    }
    file.close();
    if (mSizeToSend == 0) {
        sendFileTail();
    }
}

void FileSender::sendFileHeader(int fileSize)
{
    QFileInfo fi(mFilePath);
    mFileName = fi.fileName();

    BlockFileHeader fileHeader;
    fileHeader.fileSize = fileSize;
    QByteArray baFileName = mFileName.toUtf8();
    baFileName.push_back(char(0)); //end of string.

    QByteArray block((const char *)&fileHeader, sizeof(BlockFileHeader));
    block.append(baFileName);

    mProtocol->sendBlock(DataUnitTypeExt::FileHeader, block.data(), block.size(), Protocol::WaitToSend);
}

void FileSender::sendFileTail()
{
    mProtocol->sendBlock(DataUnitTypeExt::FileTail, 0, 0, Protocol::WaitToSend);
}

void FileSender::send(const QString &filePath)
{
    if (mIsRunning) return;

    mFilePath = filePath;
    if(!QThreadPool::globalInstance()->tryStart(this)){
        qWarning() << "FileSender::send: failed to send " << filePath;
    }
}

void FileSender::handleFileResp(const char *block)
{
    BlockFileResp *fileResp = (BlockFileResp *)block;

    QString fileName = QString::fromUtf8(fileResp->fileName());

    if (fileResp->continued) {
        qInfo() << "FileResp: " << fileResp->received << " bytes received of " << fileName;
    } else {
        qInfo() << "FileResp Done! " << fileResp->received << " bytes received of " << fileName;
        if (fileResp->received >= mFileSize) {
            mSenderHandler->handleSendFileFinished(fileName);
        }
    }

}

double FileSender::getSendProgress()
{
    return (mFileSize - mSizeToSend) / (double)mFileSize;
}

bool FileSender::handleReceiveBlock(uint32_t dataType, const char *block, int size)
{
    Q_UNUSED(size)
    switch (dataType) {
    case DataUnitTypeExt::FileResp:
        handleFileResp(block);
        return true;
    default:
        return false;
    }
}

void FileSender::handleReset()
{
    mIsCanceling = true;
}

}
