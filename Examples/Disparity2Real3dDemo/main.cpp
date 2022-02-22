#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"
#include "protocol.h"
#include "calibrationparams.h"

int main(int argc, char *argv[])
{
    StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
    MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

    while (!cameraA->getProtocol()->isConnected()) {
        printf("connecting...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // select tasks you want to run, you can also " enableTasks(TaskId::NoTask); " to pause all tasks
    cameraA->enableTasks(TaskId::DisplayTask);

    // request calibration parameters
    StereoCalibrationParameters params;
    cameraA->requestStereoCameraParameters(params);
    cameraHandlerA->setStereoCalibParams(params);

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
