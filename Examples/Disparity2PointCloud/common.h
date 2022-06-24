#ifndef COMMON_H
#define COMMON_H

struct FileSizeByBitNum
{
    enum Enumeration
    {
        Bit1 = 848704,
        Bit2 = 1060880,
        Bit4 = 1273056,
        Bit5 = 1843200
    };
};

struct FileFormat
{
    enum Enumeration
    {
        dat = 0,
        png
    };
};

#endif //COMMON_H
