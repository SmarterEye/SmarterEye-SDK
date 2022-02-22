#ifndef MYIMAGEHANDLER_H
#define MYIMAGEHANDLER_H
#include "camerahandler.h"
#include <string>

class MyCameraHandler: public CameraHandler
{
public:
    MyCameraHandler(std::string name);
    void handleRawFrame(const RawImageFrame *rawFrame);

protected:
    void processFrame(int frameId, const unsigned char *image, int64_t time, int width, int height, int frameFormat);

private:
    std::string mName;
};

#endif // MYIMAGEHANDLER_H
