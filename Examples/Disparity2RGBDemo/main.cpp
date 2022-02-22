#include <iostream>
#include <string>
#include <cstdio>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"

int main(int argc, char *argv[])
{
    StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
    MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

    // select tasks you want to run, you can also " enableTasks(TaskId::NoTask); " to pause all tasks
    cameraA->enableTasks(TaskId::DisplayTask);

    // you can request more data, like: FrameId::CalibLeftCamera | FrameId::Disparity
    cameraA->requestFrame(cameraHandlerA, FrameId::Disparity);

    // if you want to connect more device, create another one:
//    StereoCamera *cameraB = StereoCamera::connect("192.168.20.100");
//    MyCameraHandler *cameraHandlerB = new MyCameraHandler("camera B");
//    cameraB->requestFrame(cameraHandlerB, FrameId::CalibRightCamera);

    // prevent app to exit.
    int c = 0;
    while (c!= 'x')
    {
        c = getchar();
    }
}
