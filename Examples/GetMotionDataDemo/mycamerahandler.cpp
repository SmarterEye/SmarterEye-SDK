#include <iostream>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"
#include "frameid.h"
#include "frameext.h"

MyCameraHandler::MyCameraHandler(std::string name):
    mName(name)
{}

void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
    std::cout << "camera handler got frame id:" << rawFrame->frameId<<std::endl;
    processFrame(rawFrame->frameId, (char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame),
                 rawFrame->index);
}

void MyCameraHandler::processFrame(int frameId, const char *extended, size_t size)
{
    switch (frameId) {
    case FrameId::CalibLeftCamera:
    {
        //analyse the motion data from frame extended area.
        const FrameDataExtHead *header = reinterpret_cast<const FrameDataExtHead *>(extended);
        if (header->dataType == FrameDataExtHead::Compound) {
            header = reinterpret_cast<const FrameDataExtHead *>(header->data);
            size -= sizeof (FrameDataExtHead);
        }
        do{
            if (header->dataType == FrameDataExtHead::MotionData) {

                const MotionData *motionPtr = reinterpret_cast<const MotionData *>(header->data);
                int totalNum = (header->dataSize - sizeof(FrameDataExtHead)) / sizeof(MotionData);
                for(int index = 0; index < totalNum; index++){
                    MotionData data(motionPtr[index].accelX,motionPtr[index].accelY,motionPtr[index].accelZ,
                                    motionPtr[index].gyroX,motionPtr[index].gyroY,motionPtr[index].gyroZ,motionPtr[index].timestamp);
                    addMotionData(data);
                }
            }
            header = reinterpret_cast<const FrameDataExtHead*>(reinterpret_cast<const char*>(header) + header->dataSize);
        }while(reinterpret_cast<const char*>(header) < (extended + size));
    }
        break;
    default:
        break;
    }
}
