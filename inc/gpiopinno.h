#ifndef GPIONO_H
#define GPIONO_H

struct GpioPinNo
{
    enum Enumeration
    {
        Unused  = 0,
        RedStandbyLed = 164,
        GreenStandbyLed = 165,
        RedLed = 963,
        RightLandLed = 975,
        FcwRedLed = 960,
        GreenLed = 964,
        ExtendedKey1 = 965,
        ExtendedKey2 = 966,
        ExtendedKey3 = 967,
        ExtendedKey4 = 968,
        ExtendedKey5 = 969,
        CameraPower = 972,
        VanPower = 973,
        BeepPower = 974,
        SmallScreen = 978,
        Direction485GpsOrObd = 979,
        BeepPower_1 = 981,
        BeepPower_2 = 980,
        TurnSingal = 983,
        PowerSingal = 984,
        PowerHolder = 985,
        SensorLDOEnable = 991,
    };
};

#endif // GPIONO_H
