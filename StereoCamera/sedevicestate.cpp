#include "sedevicestate.h"

static const std::map<int, std::string> kErrorStringTable
{
    {0,     "No error"},
    {2,     "Camera:        sensor error"},
    {4,     "Camera:        internal watchdog timer error"},
    {5,     "Camera:        PLDDR error"},
    {6,     "Camera:        no remap data"},
    {7,     "Camera:        ISP error"},
    {8,     "Camera:        unknown error"},
    {11,    "Camera:        disparity unqualified"},
    {21,    "VTDCP:         device invalid"},
    {22,    "VTDCP:         cable not connected"},
    {23,    "VTDCP:         remote refuse"},
    {24,    "VTDCP:         host not found"},
    {25,    "VTDCP:         network resource error"},
    {26,    "VTDCP:         connect timeout error"},
    {27,    "VTDCP:         network error"},
    {28,    "VTDCP:         unknown error"},
    {31,    "CanInterface:  vehicle signal receive error"},
    {32,    "CanInterface:  sensor signal receive error"},
    {33,    "CanInterface:  channel0 bus error"},
    {34,    "CanInterface:  channel1 bus error"},
    {41,    "Sensor:        motion sensor error"},
    {45,    "Sensor:        gps sensor error"},
    {46,    "Sensor:        4G sensor error"},
    {81,    "Hardware:      high temperature error"},
    {91,    "Image:         image smudge error"}
};

SEDeviceState::SEDeviceState()
{
    mDeviceState = SEDeviceState::NormalState;
    mErrorCodeList.clear();
    mIsHighTemperature = false;
}

SEDeviceState::SEDeviceState(const SEDeviceState &state_)
{
    mErrorCodeList.clear();
    for (uint8_t i =0; i < state_.getErrorCodeCount();i++) {
        mErrorCodeList.push_back(state_.errorCode(i));
    }
    mDeviceState = state_.currentState();
    mIsHighTemperature = state_.deviceHighTemperature();
}

SEDeviceState::DeviceState SEDeviceState::currentState() const
{
    return mDeviceState;
}

uint8_t SEDeviceState::getErrorCodeCount() const
{
    return (uint8_t)mErrorCodeList.size();
}

int SEDeviceState::errorCode(uint8_t index) const{
    if(mErrorCodeList.size() == 0) return 0;

    if(index <= 0 || index >= mErrorCodeList.size()){
        return mErrorCodeList[0];
    }else{
        return mErrorCodeList[index];
    }
}

std::string SEDeviceState::errormsg(int errCode) const{
    std::map<int, std::string>::const_iterator iter;
    iter = kErrorStringTable.find(errCode);
    std::string errmsg = "Error "+std::to_string(errCode)+": Unknown error";
    if(iter != kErrorStringTable.end()){
        errmsg = iter->second;
    }
    return errmsg;
}

void SEDeviceState::setDeviceState(SEDeviceState::DeviceState state)
{
    mDeviceState = state;
}

void SEDeviceState::setErrorCodeList(const int *list)
{
    bool highTemp = false;
    mErrorCodeList.clear();
    for(int i=0;i<128;i++){
        if(list[i] == 0){
            break;
        }else if(list[i] == 81){
            highTemp = true;
        }
        mErrorCodeList.push_back(list[i]);
    }
    if(mIsHighTemperature != highTemp){
        mIsHighTemperature = highTemp;
    }
}

void SEDeviceState::clearErrorCodeList()
{
    mErrorCodeList.clear();
}
