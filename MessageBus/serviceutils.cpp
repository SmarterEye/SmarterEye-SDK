#include "serviceutils.h"

#include <QString>

#include "service.h"
#include "message.h"
#include "messageutils.h"

ServiceUtils::ServiceUtils()
{

}

void ServiceUtils::playAudioByName(Service *service, const QString& fileName)
{
    MessagePlayAudio msgAudio;
    MessageUtils::copyQStringToMString(fileName, msgAudio.fileName);
    service->sendMessage(MessageType::PlayAudio, msgAudio);
}


