#ifndef FILESENDER_H
#define FILESENDER_H

#include "blockhandler.h"
#include "satpext_global.h"
#include <QRunnable>
#include <QString>

class QFile;

namespace SATP {

class Protocol;
class FileSenderHandler;

class SATPEXTSHARED_EXPORT FileSender : public BlockHandler, public QRunnable
{
public:
    FileSender(Protocol *protocol, FileSenderHandler *senderHandler);
    virtual ~FileSender();
    void run();
    void send(const QString &filePath);
    //override.
    bool handleReceiveBlock(uint32_t dataType, const char *block, int size);
    void handleReset();
    double getSendProgress();

protected:
    void sendFileSynchronously();
    void sendFileHeader(int fileSize);
    void sendFileTail();
    void handleFileResp(const char *block);

private:
    Protocol *mProtocol;
    FileSenderHandler  *mSenderHandler;

    QString mFilePath;
    QString mFileName;
    qint64 mFileSize;
    qint64 mSizeToSend;
    bool mIsRunning;
    bool mIsCanceling;
};

}
#endif // FILESENDER_H
