#ifndef CALIBSTATUSDEF_H
#define CALIBSTATUSDEF_H

namespace CalibStatus
{
enum Enumeration {
    NoCalib         = 0,
    PrimaryCalib    = 1 << 0,
    AdvancedCalib   = 1 << 1,
    OutdoorCalib    = 1 << 2,
    AutoCalib       = 1 << 3
    };
}

#endif // CALIBSTATUSDEF_H


