#ifndef METAIDDEF
#define METAIDDEF

struct MetaId
{
    enum Enumeration {
        LaneInfo           = 1 << 16,
        ObstacleRect       = 1 << 17,
    };
};

#endif // METAIDDEF

