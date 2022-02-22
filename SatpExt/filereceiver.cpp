#include "filereceiver.h"

#include <QFile>
#include <QDebug>
#include <QDir>

#include "protocol.h"
#include "filereceiverhandler.h"
#include "satpext.h"

namespace SATP
{

FileReceiver::FileReceiver(Protocol *protocol):
    mProtocol(protocol),
    mFile(nullptr)
{
    mProtocol->registerBlockHandler(this);
}

FileReceiver::~FileReceiver()
{
    mProtocol->unregisterBlockHandler(this);
}

void FileReceiver::handleFileHeader(const char *block)
{
    const BlockFileHeader *fileHeader = reinterpret_cast<const BlockFileHeader *>(block);
    mFileSize = static_cast<int>(fileHeader->fileSize);
    QString fileName = QString::fromUtf8(fileHeader->fileName());
    QString filePath = mRecvFileDir + fileName;
    mReceived = 0;
    mPacketCount = 0;
    if (nullptr != mFile && mFile->isOpen()) {
        mFile->close();
        delete mFile;
    }
    mFile = new QFile(filePath);
    mFile->open(QIODevice::WriteOnly);
}

void FileReceiver::handleFileData(const char *block, int size)
{
    if (mFile == nullptr) return;

    mFile->write(block, size);
    mReceived += size;
    mPacketCount++;

    if (mPacketCount % 5 == 0) {
        sendFileResp();
    }
}

void FileReceiver::handleFileTail(const char *block)
{
    if (mFile == nullptr) return;
    Q_UNUSED(block)
    sendFileResp(true);
    QString filePath = mFile->fileName();
    mFile->close();
    delete mFile;
    mFile = nullptr;
    raiseReceiverHandlers(filePath);
    qInfo() << "File transfer done! " << filePath;
}

void FileReceiver::sendFileResp(bool finished)
{
    BlockFileResp fileResp;
    fileResp.received = mReceived;
    fileResp.continued = !finished;
    QByteArray baFileName = mFile->fileName().toUtf8();
    baFileName.push_back(char(0));//end of string.

    QByteArray block((const char *)&fileResp, sizeof(BlockFileResp));
    block.append(baFileName);

    mProtocol->sendBlock(DataUnitTypeExt::FileResp, block, block.size(), Protocol::EnqueueForcedly);
}

void FileReceiver::raiseReceiverHandlers(const QString &filePath)
{
    for (FileReceiverHandler *handler : mReceiverHandlers) {
        handler->handleReceiveFile(filePath);
    }
}

bool FileReceiver::handleReceiveBlock(uint32_t dataType, const char *block, int size)
{
    switch (dataType) {
    case DataUnitTypeExt::FileHeader:
        handleFileHeader(block);
        return true;
    case DataUnitTypeExt::FileTail:
        handleFileTail(block);
        return true;
    case DataUnitTypeExt::FileData:
        handleFileData(block, size);
        return true;
    default:
        return false;
    }
}

void FileReceiver::registerReceiverHandler(FileReceiverHandler *receiverHandler)
{
    if (receiverHandler != nullptr) {
        mReceiverHandlers << receiverHandler;
    }
}
void FileReceiver::unregisterReceiverHandler(FileReceiverHandler *receiverHandler)
{
    if (receiverHandler != nullptr) {
        mReceiverHandlers.remove(mReceiverHandlers.indexOf(receiverHandler));
    }
}

double FileReceiver::getReceiveProgress()
{
    return (double)mReceived / mFileSize;
}

void FileReceiver::setRecvFileDir(const QString &dir) {
    mRecvFileDir = dir;
    if (!mRecvFileDir.endsWith("/")) {
        mRecvFileDir.append("/");
    }

    QDir checkDir(dir);
    if (!checkDir.exists()) {
        checkDir.mkpath(".");
    }
}

}
