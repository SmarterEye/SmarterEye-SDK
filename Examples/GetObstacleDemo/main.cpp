#include <iostream>
#include <string>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "mycamerahandler.h"
#include "calibrationparams.h"

int main(int argc, char *argv[])
{
    StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
    MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

    cameraA->enableTasks(TaskId::DisplayTask | TaskId::ObstacleTask);

    cameraA->requestFrame(cameraHandlerA, FrameId::Obstacle);


    // prevent app to exit.
    int c = 0;
    while (c!= 'x')
    {
        c = getchar();
    }
}
