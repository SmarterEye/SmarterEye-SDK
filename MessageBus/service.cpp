#include <QString>
#include <QDateTime>
#include <QObject>
#include <QTimer>

#include "service.h"
#include "messagerouter.h"
#include "message.h"
#include "messageutils.h"


Service::Service()
{

}

void Service::sendMessage(int type, const char* message, int size)
{
    MessageHead *head = (MessageHead *)message;
    head->time = QDateTime::currentDateTime().currentMSecsSinceEpoch();
    routeMessage(type, message, size);
}

void Service::sendMessage(int type)
{
    MessageNull msgNull;
    sendMessage(type, msgNull);
}

void Service::writeLog(QString log, int tag)
{
    MessageGneralLog msgLog;
    MessageUtils::copyQStringToMString(log, msgLog.log);
    msgLog.tag = tag;
    sendMessage(MessageType::GneralLog, msgLog);
}

void Service::messageArrive(int type, const char* message, int size)
{
    //TODO check subscription here;
    handleMessage(type, message, size);
}

void Service::postMessage(int type, QObject *context)
{
    QTimer::singleShot(0, context, [this, type](){
         sendMessage(type);
    });
}

