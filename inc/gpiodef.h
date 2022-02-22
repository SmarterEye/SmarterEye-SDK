#ifndef GPIODEF_H
#define GPIODEF_H

struct GpioState
{
    enum Enumeration
    {
        Undefined = -1,
        Low  = 0,
        High = 1
    };
};

struct GpioCommandType
{
    enum Enumeration
    {
        Off  = 0,
        On = 1,
        Flash = 2
    };
};

struct GpioEvent
{
    enum Enumeration
    {
        Unused  = 0,
        Pressed = 1,
        Released = 2,
        LongReleased = 3,
    };
};

#endif // GPIODEF_H
