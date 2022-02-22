#include <iostream>
#include <opencv2/highgui.hpp>
#include <vector>

#include "stereocamera.h"
#include "framemonitor.h"
#include "frameid.h"
#include "taskiddef.h"
#include "roadwaypainter.h"
#include "LdwDataInterface.h"

/**
 * @brief main
 * @note the sdk base on qt
 * @note on linux x86 desktop platform, there are some problems to use opencv_highgui module without qt event loop
 * @note here, calling functions of highgui in qt event loop is correct, called by invokeInLoopThread() by class StereoCamera
 */


	
int main(int argc, char *argv[])
{

    StereoCamera *camera = StereoCamera::connect("192.168.1.251");
    std::unique_ptr<FrameMonitor> frameMonitor(new FrameMonitor);

    camera->enableTasks(TaskId::DisplayTask | TaskId::LaneTask);
    camera->requestFrame(frameMonitor.get(), FrameId::CalibLeftCamera | FrameId::LaneExt);

    std::function<int()> draw_func = [&frameMonitor]() -> int {

        cv::Mat leftMat = frameMonitor->getFrameMat(FrameId::CalibLeftCamera);
        LdwDataPack ldwDataPack = frameMonitor->getFrameLane(FrameId::LaneExt);//获取车道线数据包
       
        RoadwayPainter::paintRoadway((void*)(&ldwDataPack), leftMat.data, 640, 360);//叠加左图和车道线信息
	
        cv::imshow("LaneExt",leftMat);//绘制叠加后的图像
		
		return cv::waitKey(80);

    };

    camera->invokeInLoopThread([]{
        cv::namedWindow("LaneExt");
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



