#include <iostream>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"
#include "frameid.h"
#include "disparityconvertor.h"

static const int kMinValidDisparityValue = 0;
static const int kMaxValidDisparityValue = 45;

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
    processFrame(rawFrame->frameId, (const unsigned char*)rawFrame + sizeof(RawImageFrame),
                 rawFrame->time, rawFrame->width, rawFrame->height, rawFrame->format);
}

void MyCameraHandler::processFrame(int frameId, const unsigned char *image,int64_t time, int width, int height, int frameFormat)
{
    switch (frameId) {
    case FrameId::Disparity:
    {
        int bitNum = DisparityConvertor::getDisparityBitNum(frameFormat);
        static unsigned char *rgbBuf = new unsigned char[width * height * 3];
        static float *floatData = new float[width * height];

        DisparityConvertor::convertDisparity2FloatFormat(image, width, height, bitNum, floatData);
        DisparityConvertor::convertDisparity2RGB(floatData, width, height,
                                                 kMinValidDisparityValue, kMaxValidDisparityValue, rgbBuf);
    }
        break;
    default:
        break;
    }
}
