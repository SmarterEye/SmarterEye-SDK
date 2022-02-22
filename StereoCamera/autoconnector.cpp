#include "autoconnector.h"
#include <QTimerEvent>
#include <QTimer>
#include "stereocameraimpl.h"

static const int kReconnectInvervalMS = 2000;

AutoConnector::AutoConnector(StereoCameraImpl *camera, QObject *parent) :
    QObject(parent),
    mCamera(camera)
{
    mReconnectTimerId = startTimer(kReconnectInvervalMS);
    loadIpList();
}

void AutoConnector::loadIpList()
{
    mIpList
        << "192.168.100.100"
        << "127.0.0.1"
        << "192.168.20.100"
        << "192.168.20.101"
        << "192.168.1.251"
        << "192.168.2.150"
        << "192.168.10.21"
        << "192.168.3.150";
}

void AutoConnector::timerEvent(QTimerEvent *event)
{
    if (event->timerId() == mReconnectTimerId) {
        connectServer();
    }
}

void AutoConnector::connectServer()
{
    if (mCamera->isConnected()) return;
    QString addr = mCamera->getAddress();
    if (!addr.isEmpty()
        && !mIpList.contains(addr)
        && !mIpListTryied.contains(addr)) {
        mIpList << addr;
    }

    if (mIpList.isEmpty()) {
        mIpList = mIpListTryied;
        mIpListTryied.clear();
    }

    addr =  mIpList.takeFirst();
    mCamera->connectTo(addr);
    mIpListTryied << addr;
}
