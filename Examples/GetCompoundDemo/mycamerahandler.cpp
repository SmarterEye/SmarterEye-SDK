#include <iostream>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"
#include "frameid.h"
#include "roadwaypainter.h"
#include "obstaclepainter.h"
#include "frameext.h"

MyCameraHandler::MyCameraHandler(std::string name):
    mName(name)
{}

void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
//    std::cout << mName
//              << ", got image, id: " << rawFrame->frameId
//              << " , time stamp: " << rawFrame->time
//              << std::endl;

    // put you image processing logic here.    eg:
    processFrame(rawFrame->frameId, (char*)rawFrame + sizeof(RawImageFrame),
                 (char*)rawFrame + rawFrame->dataSize + sizeof(RawImageFrame),
                 rawFrame->time, rawFrame->width, rawFrame->height);
}

void MyCameraHandler::processFrame(int frameId, char *image, char *extended, int64_t time, int width, int height)
{
    switch (frameId) {
    case FrameId::Compound:
    {
        FrameDataExtHead *header = reinterpret_cast<FrameDataExtHead *>(extended);
        static unsigned char *rgbBuf = new unsigned char[width * height * 3];
        RoadwayPainter::imageGrayToRGB((unsigned char*)image, rgbBuf, width, height);
        ObstaclePainter::paintObstacle(header->data, rgbBuf, width, height, true, false);
    }
        break;
    default:
        break;
    }
}
