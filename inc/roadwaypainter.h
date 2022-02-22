#ifndef ROADWAYPAINTER_H
#define ROADWAYPAINTER_H

#include <string.h>
#include <cmath>
#include <float.h>
#include "smartpainterdef.h"

class SmartRgbImage
{
public:
    SmartRgbImage(unsigned char *imgData, int _width, int _height);
    ~SmartRgbImage();

    int mWidth;
    int mHeight;
    int mStep;

    unsigned char *mData;
};

struct LdwLane;
struct LdwDataPack;

class SMARTPAINTER_SHARED_EXPORT RoadwayPainter
{
public:
    RoadwayPainter();
    ~RoadwayPainter();

    static bool paintRoadway(void *_roadwayParam, unsigned char * _rgbImageData, int _width_, int _height, bool maskMode = false);
    static void imageGrayToRGB(const unsigned char *gray, unsigned char *rgb, int _width, int _height);
protected:
    static void outputParamFromMToMm(void *_roadwayParam);
    static void getBeginAndEndImageIndex(int &_bH, int &_eH, int _arrayIndex[], int closeH);
    static bool paintLaneBoundary(SmartRgbImage &_img, int _bH, int _eH,
        int _lEdge[], int _rEdge[], int _alpha, int _color[], bool maskMode = false);
    static bool calculateImageIndex(void *_lens, float _worldX, float _worldY, float & _imageX, float & _imageY);
    static void getImageCurveIndex(void *_lens, void *_boundary, float _worldY, float & _imageX, float & _imageY);
    static void calculateCurveIndex(void *_lens, void *_boundary, SmartRgbImage &_image, float _by, float _ey, int _boundaryIndex[]);
    static void doCurveInterpolation(int _bH, int _eH, int _curveEquation[]);
    static void getLineEquation(float _x1, float _y1, float _x2, float _y2, float &_k, float &_b);
    static void drawLine(SmartRgbImage &_img, int _bh, int _eh, float _k, float _b,int _color[]);

    static void getImagePointFormWorldCoordinate(LdwDataPack & _dataPack,
        float _world_x, float _word_y, float & _img_x, float & _img_y);

    static void getConstParams(LdwDataPack & _dataPack, float & _alpha, float & _gamma);

    static float getEquationValue(int _degree, float _coefficient[], float _variate);
    static float getEquationSlope(int _degree, float _coefficient[], float _variate);

    static void getLaneMarkBoundary(LdwDataPack & _dataPack, float _far_distance,
        int _img_height, int _degree, float _coefficient[], int *_boundary);

    static float getFarDistance(LdwDataPack & _dataPack);

    static void paintLane(LdwDataPack & _dataPack, LdwLane & _lane,
        SmartRgbImage & _smart_image, float _far_distance, int _color[],
        bool _mask_mode);

    static void drawWhenManualLearnMode(LdwDataPack &_dataPack,
        SmartRgbImage &_smartImage, int _color[]);
};

#endif // ROADWAYPAINTER_H
