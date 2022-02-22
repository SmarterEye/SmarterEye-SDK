#ifndef AUTOCONNECTOR_H
#define AUTOCONNECTOR_H

#include <QObject>
#include "stereocamera_global.h"

class StereoCameraImpl;

class STEREOCAMERASHARED_EXPORT AutoConnector : public QObject
{
    Q_OBJECT
public:
    explicit AutoConnector(StereoCameraImpl *camera, QObject *parent = nullptr);
    void timerEvent(QTimerEvent *event);

signals:

public slots:

protected:
    void loadIpList();
    void connectServer();

private:
    StereoCameraImpl *mCamera;
    int mReconnectTimerId;
    QStringList mIpList;
    QStringList mIpListTryied;
};

#endif // AUTOCONNECTOR_H
