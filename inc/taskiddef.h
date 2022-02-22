#ifndef TASKIDDEF_H
#define TASKIDDEF_H

#ifndef TASKIDHELPER_LIBRARY
struct TaskId
{
#endif
    enum Enumeration {
        NoTask           = 0,
        DisplayTask      = 1 << 0,
        ObstacleTask     = 1 << 1,
        LaneTask         = 1 << 2,
        HeightLimitTask  = 1 << 3,
        EnvMonitorTask   = 1 << 4,
        LensSmudgeTask   = 1 << 5,
        AutoMarkTask     = 1 << 6,
        AllTasks         = 0xFFFF    // add tasks before here
    };
#ifndef TASKIDHELPER_LIBRARY
};
#else
    Q_ENUM(Enumeration)
#endif // TASKIDHELPER_LIBRARY

#endif // TASKIDDEF_H


