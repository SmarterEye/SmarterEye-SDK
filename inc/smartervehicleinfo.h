#ifndef SMARTERVEHICLEINFO_H
#define SMARTERVEHICLEINFO_H

#pragma pack(push, 1)

struct SmarterVehicleInfo {
    enum GearPosition {
        Park,
        Reverse,
        Neutral,
        Drive
    };
    float vehicleSpeed = -1.f;
    bool brakePedalOn = false;
    bool turnLightOn = false;
    bool wiperOn = false;
    float yawRate = 0.f;
    int gear = 0xF;

    bool vehicleSpeedValid = false;
    bool brakePedalValid = false;
    bool turnLightValid = false;
    bool wiperValid = false;
    bool yawRateValid = false;
    bool gearValid = false;

    void update(SmarterVehicleInfo info){
        bool flag = false;
        if (info.vehicleSpeedValid) {
            vehicleSpeedValid = true;
            vehicleSpeed = info.vehicleSpeed;
            flag = true;
        }
        if (info.brakePedalValid) {
            brakePedalValid = true;
            brakePedalOn = info.brakePedalOn;
            flag = true;
        }
        if (info.turnLightValid) {
            turnLightValid = true;
            turnLightOn = info.turnLightOn;
            flag = true;
        }
        if (info.wiperValid) {
            wiperValid = true;
            wiperOn = info.wiperOn;
            flag = true;
        }
        if (info.yawRateValid) {
            yawRateValid = true;
            yawRate = info.yawRate;
            flag = true;
        }
        if (info.gearValid) {
            gearValid = true;
            gear = info.gear;
            flag = true;
        }
        if (!flag) {
            vehicleSpeed = -1.f;
            brakePedalOn = false;
            turnLightOn = false;
            wiperOn = false;
            yawRate = 0.f;
            gear = 0xF;

            vehicleSpeedValid = false;
            brakePedalValid = false;
            turnLightValid = false;
            wiperValid = false;
            yawRateValid = false;
            gearValid = false;
        }
    }
};

#pragma pack(pop)
#endif // SMARTERVEHICLEINFO_H
