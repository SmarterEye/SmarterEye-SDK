#ifndef RECVTHREAD_H
#define RECVTHREAD_H

#include <QObject>
#include <QRunnable>

namespace SATP{
class TcpProtocol;
class RecvThread : public QObject, public QRunnable
{
    Q_OBJECT
public:
    RecvThread(TcpProtocol *parent) {
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
};

}
#endif // RECVTHREAD_H
