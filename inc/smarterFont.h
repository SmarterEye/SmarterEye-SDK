#ifndef SMARTER_FONT_H_
#define SMARTER_FONT_H_

#ifndef NULL
#define NULL 0
#endif // !NULL

#include "smartpainterdef.h"

struct smarterPoint
{
    int x;
    int y;

    smarterPoint() : x(0), y(0) {}
    smarterPoint(int x, int y) : x(x), y(y) {}
};

struct smarterSize
{
    int width;
    int height;

    smarterSize() : width(0), height(0) {}
    smarterSize(int width, int height) : width(width), height(height) {}
};

class SMARTPAINTER_SHARED_EXPORT smarterFont
{
public:
    smarterFont();
    ~smarterFont();

    static void putText(unsigned char *buffer, int width, int height,
                        const char *text, const int org[2], int channels, double fontScale = 1.0,
    const unsigned char color[3] = 0, int thickness = 1);

private:

};

extern void ThickLine(unsigned char *img, int width, int height, smarterPoint p0, smarterPoint p1, const unsigned char color[3],
int thickness, int flags, int channels);

#endif
