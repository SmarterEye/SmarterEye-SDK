#include "yuv2rgb.h"

enum {
    R = 0,
    G,
    B
};

RGB YuvToRGB::Yuv2Rgb(char Y, char U, char V)
{
    RGB rgb;
    int r = (int)((Y&0xff) + 1.4075 * ((V&0xff)-128));
    int g = (int)((Y&0xff) - 0.3455 * ((U&0xff)-128) - 0.7169*((V&0xff)-128));
    int b = (int)((Y&0xff) + 1.779 * ((U&0xff)-128));
    rgb.r =(r<0? 0: r>255? 255 : r);
    rgb.g =(g<0? 0: g>255? 255 : g);
    rgb.b =(b<0? 0: b>255? 255 : b);

    return rgb;
}

RGB YuvToRGB::YCbCr2Rgb(unsigned char Y, unsigned char Cb, unsigned char Cr)
{
    RGB rgb;
    int r = (int)(1.164 * Y + 1.596 * Cr - 222.912);
    int g = (int)(1.164 * Y - 0.391 * Cb -0.813 *Cr + 135.488);
    int b = (int)(1.164 * Y + 2.018 * Cb -276.928);
    rgb.r =(r<0? 0: r>255? 255 : r);
    rgb.g =(g<0? 0: g>255? 255 : g);
    rgb.b =(b<0? 0: b>255? 255 : b);

    return rgb;
}

char * YuvToRGB::YCbYCr2Rgb(const unsigned char* src, char* dest, int width, int height)
{
    int lineWidth = 2*width;
    int startY;
    int Y1Position;
    int Y2Position;
    int CbPosition;
    int CrPosition;
    int index;
    for(int i=0; i<height; i++){
        startY = i*lineWidth;
        for(int j = 0; j < lineWidth; j+=4){
            Y1Position = j + startY;
            Y2Position = Y1Position+2;
            CbPosition = Y1Position+1;
            CrPosition = Y1Position+3;
            index = Y1Position/2*3;
            RGB tmp = Yuv2Rgb(src[Y1Position], src[CbPosition], src[CrPosition]);
            dest[index+R] = tmp.r;
            dest[index+G] = tmp.g;
            dest[index+B] = tmp.b;
            index += 3;
            tmp = Yuv2Rgb(src[Y2Position], src[CbPosition], src[CrPosition]);
            dest[index+R] = tmp.r;
            dest[index+G] = tmp.g;
            dest[index+B] = tmp.b;
        }
    }

    return dest;
}

unsigned char * YuvToRGB::YCbYCrGetY(const unsigned char* src, unsigned char* dest, int width, int height)
{
    int lineWidth = 2*width;
    int startY;
    int Y1;
    int index;
    for(int i=0; i<height; i++){
        startY = i*lineWidth;
        for(int j = 0; j < lineWidth; j+=2){
            Y1 = j + startY;
            index = (Y1>>1);
            dest[index] = src[Y1];
        }
    }

    return dest;
}

char * YuvToRGB::YCbYCrPlannar2Rgb(const unsigned char* src, char* dest, int width, int height)
{
    const unsigned char *CbBase = src + width * height;
    const unsigned char *CrBase = CbBase + width * height / 2;
    for(int i=0; i<width*height; i+=2){
            RGB tmp = Yuv2Rgb(src[i], CbBase[i/2], CrBase[i/2]);
            dest[i*3+R] = tmp.r;
            dest[i*3+G] = tmp.g;
            dest[i*3+B] = tmp.b;
            tmp = Yuv2Rgb(src[i+1], CbBase[i/2], CrBase[i/2]);
            dest[(i+1)*3+R] = tmp.r;
            dest[(i+1)*3+G] = tmp.g;
            dest[(i+1)*3+B] = tmp.b;
    }

    return dest;
}
