#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include "stereocamera.h"
#include "protocol.h"
#include "deviceinfo.h"
#include "sedevicestate.h"

#ifdef _WINDOWS
#include <direct.h>
#else
#include <unistd.h>
#endif

using namespace std;

string readTxt(string file)
{
    ifstream infile;
    infile.open(file.data());   //将文件流对象与文件连接起来 
    //assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行 

    string sdkVersion;
    while (getline(infile, sdkVersion))
    {
        infile.close();             //关闭文件输入流 
        return sdkVersion;
    }
}

void dumpDeviceInfo(const DeviceInfo &deviceInfo, string externSdkVersion)
{
    cout << "************ Camera params ************" << endl;
    printf("Device model\n");
    cout << deviceInfo.HardwareVersion << endl;
    printf("Device serial number\n");
    cout << deviceInfo.SerialNum << endl;
    printf("Firmware type\n");
    cout << deviceInfo.ProductCode << endl;
    printf("Firmware version\n");
    cout << deviceInfo.AdasVersion << endl;
    printf("Platform version\n");
    cout << deviceInfo.PlatformVersion << endl;
    printf("Camera version\n");
    cout << deviceInfo.CameraFirmwareVersion << endl;
    printf("sdk version\n");
    cout << deviceInfo.SdkVersion << endl;
    printf("External SDK version\n");
    cout << externSdkVersion << endl;
    printf("obstacle algorithm version\n");
    cout << deviceInfo.ObsVersion << endl;
    printf("Lane line algorithm version\n");
    cout << deviceInfo.LaneVersion << endl;
    printf("Device status\n");
    cout << deviceInfo.InitialSetupStatus << endl;
    printf("cpu temperature\n");
    cout << deviceInfo.CPUTemperature << endl;
}

#ifdef _WINDOWS
#else
string getDirectory()
{
    char abs_path[1024];
    int cnt = readlink("/proc/self/exe", abs_path, 1024);//获取可执行程序的绝对路径
    if (cnt < 0 || cnt >= 1024)
    {
        return NULL;
    }

    for (int i = cnt; i >= 0; --i)
    {
        if (abs_path[i] == '/')
        {
            abs_path[i + 1] = '\0';
            break;
        }
    }

    string path(abs_path);

    return path;
}
#endif

int main(int argc, char *argv[])
{
    StereoCamera *camera = StereoCamera::connect("192.168.1.251");

    while (!camera->isConnected()) {
        printf("connecting...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));   // wait for rtdb load
    string externSdkVersion;
#ifdef _WINDOWS
    char cwd[256];
    _getcwd(cwd, 256);
    char* path;
    if ((path = _getcwd(NULL, 0)) == NULL) {
        cerr << "Error message : _getcwd error" << endl;
    }
    else {
        string sdkVersion;
        sdkVersion = "//sdkVersion";
        sdkVersion = path + sdkVersion;
        externSdkVersion = readTxt(sdkVersion);
    }
#else
    string sdkVersion;
    sdkVersion = "//sdkVersion";
    string path;
    path = getDirectory();
    sdkVersion = path + sdkVersion;
    externSdkVersion = readTxt(sdkVersion);
#endif
    SEDeviceState seDeviceState;
    seDeviceState = camera->getDeviceState();
    DeviceInfo deviceInfo;
    camera->requestDeviceInfo(deviceInfo);
    dumpDeviceInfo(deviceInfo, externSdkVersion);

    while (true)
    {
        camera->requestDeviceInfo(deviceInfo);
        cout << "cpu temperature    " << deviceInfo.CPUTemperature << endl;
        if(seDeviceState.deviceHighTemperature()){
            cout << " high temperature" << endl;
        }

        switch(seDeviceState.currentState())
        {
          case SEDeviceState::NormalState:
            cout << "device state : " << "normal state" <<endl;
            break;
          case SEDeviceState::AbnormalState:
            cout << "device state : " << "abnormal state" <<endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}
