#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/highgui.hpp>

#include "stereocamera.h"
#include "frameid.h"
#include "taskiddef.h"
#include "protocol.h"
#include "framemonitor.h"


/**
 * @brief main
 * @note the sdk base on qt
 * @note on linux x86 desktop platform, there are some problems to use opencv_highgui module without qt event loop
 * @note here, calling functions of highgui in qt event loop is correct, called by invokeInLoopThread() by class StereoCamera
 */

int main(int argc, char *argv[])
{
    StereoCamera *camera = StereoCamera::connect("192.168.1.251");
    while (!camera->isConnected()) {
        printf("connecting...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::unique_ptr<FrameMonitor> frameMonitor(new FrameMonitor);

    std::this_thread::sleep_for(std::chrono::seconds(3)); // wait for rtdb load 

    camera->enableTasks(TaskId::DisplayTask);
    camera->requestFrame(frameMonitor.get(), FrameId::DepthRGB);

    std::function<int()> draw_func = [&frameMonitor,&camera]() -> int {

        cv::Mat depthRGB = frameMonitor->getFrameMat(FrameId::DepthRGB,camera);//获取到视差图

        if (!depthRGB.empty()) {
            cv::imshow("depthRGB", depthRGB);
        }

        return cv::waitKey(80);
    };

    camera->invokeInLoopThread([]{
        cv::namedWindow("depthRGB");
        
    });

    // main thread loop for drawing images
    while (true) {
        frameMonitor->waitForFrames();  // wait for frames ready

        int key = 0;
        camera->invokeInLoopThread([&key, &draw_func]{
            key = draw_func();
        });

        if (key == 27) {
            // press Esc to close
            break;
        }
    }

    camera->invokeInLoopThread([]{
        cv::destroyAllWindows();
    });

    return 0;
}

