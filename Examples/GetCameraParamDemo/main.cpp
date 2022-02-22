#include <chrono>
#include <thread>
#include <cstdio>

#include "stereocamera.h"
#include "protocol.h"
#include "calibrationparams.h"

void dumpCameraParams(const StereoCalibrationParameters &params)
{
    printf("************ Camera params ************\n");
    printf("Focus: %e pixel\n", params.focus);
    printf("Optical center X: %e pixel\n", params.cx);
    printf("Optical center Y: %e pixel\n", params.cy);
    printf("R-vector roll: %e rad\n", params.RRoll);
    printf("R-vector pitch: %e rad\n", params.RPitch);
    printf("R-vector yaw: %e rad\n", params.RYaw);
    printf("Translation x: %e mm\n", params.Tx);
    printf("Translation y: %e mm\n", params.Ty);
    printf("Translation z: %e mm\n", params.Tz);
    printf("**************************************\n");
}

int main(int argc, char *argv[])
{
    StereoCamera *camera = StereoCamera::connect("192.168.1.251");

    while (!camera->isConnected()) {
        printf("connecting...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));   // wait for rtdb load

    StereoCalibrationParameters params;
    camera->requestStereoCameraParameters(params);

    dumpCameraParams(params);

    int c = 0;
    while (c!= 'x')
    {
        c = getchar();
    }
}
