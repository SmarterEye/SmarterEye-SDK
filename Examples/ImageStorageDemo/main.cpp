
#include "stereocamera.h"
#include "imagestorage.h"
#include "taskiddef.h"
#include <QDebug>
#include "frameid.h"
#include <QDesktopWidget>
#include <QString>
#include <iostream>


int main(int argc, char *argv[])
{
    StereoCamera * camera = StereoCamera::connect("192.168.1.251");
    std::unique_ptr<ImageStorage> mImageStorage(new ImageStorage);
    camera->requestFrame(mImageStorage.get(), FrameId::CalibLeftCamera | FrameId::Disparity);
    mImageStorage->init();
    int c = 0;
    while(c != 'x')
    {
        switch(c)
        {
         case '1':
              emit mImageStorage->captureStarted();
              std::cout << "Start capture..." <<std::endl;
            break;
         case '2':
              emit mImageStorage->captureStopped();
              std::cout << "Capture stopped." <<std::endl;
              break;
          default:
              break;
         }
       c = getchar();
    }
}
