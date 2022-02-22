#ifndef DEVICESTATUS_H
#define DEVICESTATUS_H

#include <QObject>
#include <QMutex>
#include "service.h"
#include "sedevicestate.h"

class DeviceStatus : public QObject, public Service
{
    Q_OBJECT
public:
    static DeviceStatus *instance();
    void handleMessage(int type, const char *message, int size);
    const SEDeviceState &getDeviceState();

protected:
    void handleOldWarningMessage(int type, const char *message);
private:
    explicit DeviceStatus(QObject *parent = nullptr);

    SEDeviceState *mSeDeviceState;
    bool mIsHighTemperature;
    QMutex mAccessMutex;
};

#endif // DEVICESTATUS_H
