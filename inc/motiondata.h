#ifndef MOTIONDATA
#define MOTIONDATA

#pragma pack(push, 1)

struct MotionData
{
    double accelX;
    double accelY;
    double accelZ;
    double gyroX;
    double gyroY;
    double gyroZ;
    long long timestamp;

    MotionData(){}
    MotionData(double accX, double accY, double accZ, double gyrX, double gyrY, double gyrZ, long long time)
        :accelX(accX),accelY(accY),accelZ(accZ),gyroX(gyrX),gyroY(gyrY),gyroZ(gyrZ),timestamp(time)
    {
    }
};

#pragma pack(pop)

#endif // MOTIONDATA

