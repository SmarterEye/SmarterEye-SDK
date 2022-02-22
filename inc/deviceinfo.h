#ifndef _DEVICEINFO_H_
#define _DEVICEINFO_H_

#include <string>

using namespace std;

struct DeviceInfo
{
    string HardwareVersion;                       

    string SerialNum;                          
    string ProductCode;                          

    string AdasVersion;                       
    string PlatformVersion;
    string CameraFirmwareVersion;

    string SdkVersion;                          
    string ObsVersion;
    string LaneVersion;
    string InitialSetupStatus;
    string CPUTemperature;
};

#endif //_DEVICEINFO_H_
