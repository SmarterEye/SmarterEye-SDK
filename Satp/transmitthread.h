#ifndef TRANSMITTHREAD_H
#define TRANSMITTHREAD_H
#include <QRunnable>
#include <QObject>
#include <QWaitCondition>
#include <QMutex>

namespace SATP{
class TcpProtocol;
class TransmitThread : public QObject, public QRunnable
{
    Q_OBJECT
public:
    TransmitThread(TcpProtocol *parent) {
        mParent = parent;
        mExitFlag = false;
    }
    void run();
    void exit(){
        mExitFlag = true;
    }
signals:
    void bytesWritten(qint64 bytes);

private:
    TcpProtocol *mParent;
    bool mExitFlag;
    QWaitCondition mCondition;
    QMutex mLocker;
};

}
#endif // TRANSMITTHREAD_H
