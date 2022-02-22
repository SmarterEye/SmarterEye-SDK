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
#include "pcviewer.h"

void savePCDFile1(MyCameraHandler *cameraHandlerA)
{
    std::shared_ptr<PCViewer> viewer1(new PCViewer(true));
    while (!viewer1->wasStopped()) {
        //std::cout << "savePCDFile1..." << std::endl;
        auto cloud = cameraHandlerA->getCloud();
        viewer1->update(cloud, 1);
    }
}

void savePCDFile2(MyCameraHandler *cameraHandlerA)
{
    std::shared_ptr<PCViewer> viewer2(new PCViewer(true));
    while (!viewer2->wasStopped()) {
    //std::cout << "savePCDFile2..." << std::endl;
    auto cloud = cameraHandlerA->getCloud();
    viewer2->update(cloud, 2);
    }
}


int main(int argc, char *argv[])
{
    StereoCamera *cameraA = StereoCamera::connect("192.168.1.251");
    MyCameraHandler *cameraHandlerA = new MyCameraHandler("camera A");

    while (!cameraA->getProtocol()->isConnected()) {
        std::cout << "connecting..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    cameraA->enableTasks(TaskId::DisplayTask);

    StereoCalibrationParameters params;
    cameraA->requestStereoCameraParameters(params);
    cameraHandlerA->setStereoCalibParams(params);
    cameraA->requestFrame(cameraHandlerA, FrameId::Disparity);

    std::shared_ptr<PCViewer> viewer(new PCViewer(true));

    std::thread t1(savePCDFile1, cameraHandlerA);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
	
    std::thread t2(savePCDFile2, cameraHandlerA);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));


    // main loop for showing point cloud
    while (!viewer->wasStopped()) {
        //std::cout << "main..." << std::endl;
        auto cloud = cameraHandlerA->getCloud();
        viewer->update(cloud, 0);
    }
    t1.join();
    t2.join();

    std::cout << "stopped" << std::endl;
    viewer->close();

    cameraA->disconnectFromServer();
    delete cameraA;
    delete cameraHandlerA;

    return 0;
}
