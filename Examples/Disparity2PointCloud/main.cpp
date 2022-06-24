
#include <iostream>
#include "convertormanage.h"
#include <QCoreApplication>
#include "disparityframeconvertor.h"
#include "pcviewer.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    std::cout << "Input the path of your local image data: " << std::endl;
    std::string disparityPath;
    std::cin >> disparityPath;
    QString inputPath = QString::fromLocal8Bit(disparityPath.c_str());
    QString selectedPath = inputPath.replace("\\","/");
    DisparityFrameConvertor * disparityFrameConvertor = new DisparityFrameConvertor;
    ConvertorManage *mConvertorManager = new ConvertorManage(selectedPath,disparityFrameConvertor);
    if(!mConvertorManager->requestStereoCameraParams())
    {
       printf("Requset stereo camera calibration parameters failed!\n");
       while(1);
    }
    printf("Start convert...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    mConvertorManager->start();
    std::shared_ptr<PCViewer> viewer(new PCViewer(true));
    while(!viewer->wasStopped())
    {
        if(!mConvertorManager->isRunning() && disparityFrameConvertor->GetQueueSize() == 0)
        {
            printf("All saved!\n");
            break;
        }

        auto cloud = disparityFrameConvertor->getCloud();
        viewer->save_dir_ = disparityFrameConvertor->GetCloudPath().toStdString();
        viewer->update(cloud);
    }
    return a.exec();
}
