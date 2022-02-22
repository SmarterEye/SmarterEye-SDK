#include <vector>
#include <sstream>
#include <algorithm>

#include "roadwaypainter.h"
#include "LdwDataInterface.h"

static unsigned char g_yetColor[] = {
    131,0,0,135,0,0,139,0,0,143,0,0,147,0,0,151,0,0,155,0,0,159,0,0,163,0,0,167,0,0,171,
    0,0,175,0,0,179,0,0,183,0,0,187,0,0,191,0,0,195,0,0,199,0,0,203,0,0,207,0,0,211,0,0,
    215,0,0,219,0,0,223,0,0,227,0,0,231,0,0,235,0,0,239,0,0,243,0,0,247,0,0,251,0,0,255,
    0,0,255,4,0,255,8,0,255,12,0,255,16,0,255,20,0,255,24,0,255,28,0,255,32,0,255,36,0,
    255,40,0,255,44,0,255,48,0,255,52,0,255,56,0,255,60,0,255,64,0,255,68,0,255,72,0,255,
    76,0,255,80,0,255,84,0,255,88,0,255,92,0,255,96,0,255,100,0,255,104,0,255,108,0,255,
    112,0,255,116,0,255,120,0,255,124,0,255,128,0,255,131,0,255,135,0,255,139,0,255,143,
    0,255,147,0,255,151,0,255,155,0,255,159,0,255,163,0,255,167,0,255,171,0,255,175,0,255,
    179,0,255,183,0,255,187,0,255,191,0,255,195,0,255,199,0,255,203,0,255,207,0,255,211,0,
    255,215,0,255,219,0,255,223,0,255,227,0,255,231,0,255,235,0,255,239,0,255,243,0,255,
    247,0,255,251,0,255,255,0,251,255,4,247,255,8,243,255,12,239,255,16,235,255,20,231,255,
    24,227,255,28,223,255,32,219,255,36,215,255,40,211,255,44,207,255,48,203,255,52,199,
    255,56,195,255,60,191,255,64,187,255,68,183,255,72,179,255,76,175,255,80,171,255,84,167,
    255,88,163,255,92,159,255,96,155,255,100,151,255,104,147,255,108,143,255,112,139,255,
    116,135,255,120,131,255,124,128,255,128,124,255,131,120,255,135,116,255,139,112,255,143,
    108,255,147,104,255,151,100,255,155,96,255,159,92,255,163,88,255,167,84,255,171,80,255,
    175,76,255,179,72,255,183,68,255,187,64,255,191,60,255,195,56,255,199,52,255,203,48,255,
    207,44,255,211,40,255,215,36,255,219,32,255,223,28,255,227,24,255,231,20,255,235,16,255,
    239,12,255,243,8,255,247,4,255,251,0,255,255,0,251,255,0,247,255,0,243,255,0,239,255,0,
    235,255,0,231,255,0,227,255,0,223,255,0,219,255,0,215,255,0,211,255,0,207,255,0,203,255,
    0,199,255,0,195,255,0,191,255,0,187,255,0,183,255,0,179,255,0,175,255,0,171,255,0,167,
    255,0,163,255,0,159,255,0,155,255,0,151,255,0,147,255,0,143,255,0,139,255,0,135,255,0,
    131,255,0,128,255,0,124,255,0,120,255,0,116,255,0,112,255,0,108,255,0,104,255,0,100,255,
    0,96,255,0,92,255,0,88,255,0,84,255,0,80,255,0,76,255,0,72,255,0,68,255,0,64,255,0,60,
    255,0,56,255,0,52,255,0,48,255,0,44,255,0,40,255,0,36,255,0,32,255,0,28,255,0,24,255,0,
    20,255,0,16,255,0,12,255,0,8,255,0,4,255,0,0,255,0,0,251,0,0,247,0,0,243,0,0,239,0,0,235,
    0,0,231,0,0,227,0,0,223,0,0,219,0,0,215,0,0,211,0,0,207,0,0,203,0,0,199,0,0,195,0,0,191,
    0,0,187,0,0,183,0,0,179,0,0,175,0,0,171,0,0,167,0,0,163,0,0,159,0,0,155,0,0,151,0,0,147,
    0,0,143,0,0,139,0,0,135,0,0,131,0,0,128 };
static unsigned char g_yetColorInMaskMode[] = {
    16,0,124,16,0,124,17,0,124,17,0,124,18,0,124,18,0,124,19,0,124,19,0,124,20,0,124,20,0,124,
    21,0,124,21,0,124,22,0,124,22,0,124,23,0,124,23,0,124,24,0,124,24,0,124,25,0,124,25,0,124,
    26,0,124,26,0,124,27,0,124,27,0,124,28,0,124,28,0,124,29,0,124,29,0,124,30,0,124,30,0,124,
    31,0,124,31,0,124,31,0,124,95,0,124,95,0,124,159,0,124,159,0,124,223,0,124,223,0,124,31,1,124,
    31,1,124,95,1,124,95,1,124,159,1,124,159,1,124,223,1,124,223,1,124,31,2,124,31,2,124,95,2,124,
    95,2,124,159,2,124,159,2,124,223,2,124,223,2,124,31,3,124,31,3,124,95,3,124,95,3,124,159,3,124,
    159,3,124,223,3,124,223,3,124,31,4,124,31,4,124,31,4,124,95,4,124,95,4,124,159,4,124,159,4,124,
    223,4,124,223,4,124,31,5,124,31,5,124,95,5,124,95,5,124,159,5,124,159,5,124,223,5,124,223,5,124,
    31,6,124,31,6,124,95,6,124,95,6,124,159,6,124,159,6,124,223,6,124,223,6,124,31,7,124,31,7,124,
    95,7,124,95,7,124,159,7,124,159,7,124,223,7,124,223,7,124,223,7,124,222,23,124,222,23,124,221,39,124,
    221,39,124,220,55,124,220,55,124,219,71,124,219,71,124,218,87,124,218,87,124,217,103,124,217,103,124,216,119,124,
    216,119,124,215,135,124,215,135,124,214,151,124,214,151,124,213,167,124,213,167,124,212,183,124,212,183,124,211,199,124,
    211,199,124,210,215,124,210,215,124,209,231,124,209,231,124,208,247,124,208,247,124,208,7,125,207,7,125,207,7,125,
    206,23,125,206,23,125,205,39,125,205,39,125,204,55,125,204,55,125,203,71,125,203,71,125,202,87,125,202,87,125,
    201,103,125,201,103,125,200,119,125,200,119,125,199,135,125,199,135,125,198,151,125,198,151,125,197,167,125,197,167,125,
    196,183,125,196,183,125,195,199,125,195,199,125,194,215,125,194,215,125,193,231,125,193,231,125,192,247,125,192,247,125,
    192,247,125,128,247,125,128,247,125,64,247,125,64,247,125,0,247,125,0,247,125,192,246,125,192,246,125,128,246,125,
    128,246,125,64,246,125,64,246,125,0,246,125,0,246,125,192,245,125,192,245,125,128,245,125,128,245,125,64,245,125,
    64,245,125,0,245,125,0,245,125,192,244,125,192,244,125,128,244,125,128,244,125,64,244,125,64,244,125,0,244,125,
    0,244,125,0,244,125,192,243,125,192,243,125,128,243,125,128,243,125,64,243,125,64,243,125,0,243,125,0,243,125,
    192,242,125,192,242,125,128,242,125,128,242,125,64,242,125,64,242,125,0,242,125,0,242,125,192,241,125,192,241,125,
    128,241,125,128,241,125,64,241,125,64,241,125,0,241,125,0,241,125,192,240,125,192,240,125,128,240,125,128,240,125,
    64,240,125,64,240,125,0,240,125,0,240,125,0,240,125,0,224,125,0,224,125,0,208,125,0,208,125,0,192,125,
    0,192,125,0,176,125,0,176,125,0,160,125,0,160,125,0,144,125,0,144,125,0,128,125,0,128,125,0,112,125,
    0,112,125,0,96,125,0,96,125,0,80,125,0,80,125,0,64,125,0,64,125,0,48,125,0,48,125,0,32,125,
    0,32,125,0,16,125,0,16,125,0,0,125,0,0,125,0,0,125 };

static unsigned char g_greenColor[] = { 0, 255, 0 };
static unsigned char g_redColor[] = { 255, 0, 0 };
static unsigned char g_yellowColor[] = { 255, 255, 0 };
static unsigned char g_blueColor[] = { 0, 170, 255 };

static unsigned char g_greenColorInMaskMode[] = { 124, 7, 192 };
static unsigned char g_redColorInMaskMode[] = { 125, 240, 0 };
static unsigned char g_yellowColorInMaskMode[] = { 125, 247, 192 };
static unsigned char g_blueColorInMaskMode[] = { 124, 7, 255 };

SmartRgbImage::SmartRgbImage(unsigned char *imgData, int _width, int _height)
{
    mData = imgData;
    mWidth = _width;
    mHeight = _height;
    mStep = _width * 3;
}

SmartRgbImage::~SmartRgbImage()
{
}

#define ROADWAY_COLOR_NUM 10

bool RoadwayPainter::paintRoadway(void *_roadwayParam,
    unsigned char * _rgbImageData, int _width_, int _height, bool maskMode)
{
    LdwDataPack *dataPack = (LdwDataPack *)_roadwayParam;

    outputParamFromMToMm(_roadwayParam);

    SmartRgbImage smartImage(_rgbImageData, _width_, _height);

    int color[3], lColor[3], rColor[3], adjacent_color[3];

    unsigned char *yetColor, *greenColor, *redColor, *yellowColor, *blueColor;

    if (maskMode) {
        yetColor = g_yetColorInMaskMode;
        greenColor = g_greenColorInMaskMode;
        redColor = g_redColorInMaskMode;
        yellowColor = g_yellowColorInMaskMode;
        blueColor = g_blueColorInMaskMode;
    }
    else {
        yetColor = g_yetColor;
        greenColor = g_greenColor;
        redColor = g_redColor;
        yellowColor = g_yellowColor;
        blueColor = g_blueColor;
    }

    if (!dataPack->roadway.isTracking) return false;

    if (LDW_SOFT_MANUAL_LEARNING_MODE1 == dataPack->softStatus)
    {
        return true;
    }
    else if (LDW_SOFT_MANUAL_LEARNING_MODE0 == dataPack->softStatus)
    {
        color[0] = greenColor[0];
        color[1] = greenColor[1];
        color[2] = greenColor[2];

        drawWhenManualLearnMode(*dataPack, smartImage, color);

        return true;
    }

    if (LDW_NORMAL_STEER == dataPack->steerStatus)
    {
        lColor[0] = blueColor[0]; lColor[1] = blueColor[1]; lColor[2] = blueColor[2];
        rColor[0] = blueColor[0]; rColor[1] = blueColor[1]; rColor[2] = blueColor[2];
    }
    else if (LDW_STEER_WARNING_LEFT_ == dataPack->steerStatus)
    {
        lColor[0] = redColor[0]; lColor[1] = redColor[1]; lColor[2] = redColor[2];
        rColor[0] = blueColor[0]; rColor[1] = blueColor[1]; rColor[2] = blueColor[2];
    }
    else if (LDW_STEER_WARNING_RIGHT == dataPack->steerStatus)
    {
        lColor[0] = blueColor[0]; lColor[1] = blueColor[1]; lColor[2] = blueColor[2];
        rColor[0] = redColor[0]; rColor[1] = redColor[1]; rColor[2] = redColor[2];
    }
    else if (LDW_STEER_ON_LEFT__LANE == dataPack->steerStatus)
    {
        lColor[0] = yellowColor[0]; lColor[1] = yellowColor[1]; lColor[2] = yellowColor[2];
        rColor[0] = blueColor[0]; rColor[1] = blueColor[1]; rColor[2] = blueColor[2];
    }
    else if (LDW_STEER_ON_RIGHT_LANE == dataPack->steerStatus)
    {
        lColor[0] = blueColor[0]; lColor[1] = blueColor[1]; lColor[2] = blueColor[2];
        rColor[0] = yellowColor[0]; rColor[1] = yellowColor[1]; rColor[2] = yellowColor[2];
    }

    adjacent_color[0] = blueColor[0];
    adjacent_color[1] = blueColor[1];
    adjacent_color[2] = blueColor[2];

    float far_distance = getFarDistance(*dataPack);

    if (LDW_LANE_STYLE_NONE_LANE != dataPack->roadway.left_Lane.style &&
        LDW_LANE_STYLE_PREDICT_LANE != dataPack->roadway.left_Lane.style)
    {
        paintLane(*dataPack, dataPack->roadway.left_Lane,
            smartImage, far_distance, lColor, maskMode);
    }

    if (LDW_LANE_STYLE_NONE_LANE != dataPack->roadway.rightLane.style &&
        LDW_LANE_STYLE_PREDICT_LANE != dataPack->roadway.rightLane.style)
    {
        paintLane(*dataPack, dataPack->roadway.rightLane,
            smartImage, far_distance, rColor, maskMode);
    }

    if (LDW_LANE_STYLE_NONE_LANE != dataPack->roadway.adjacentLeft_Lane.style &&
        LDW_LANE_STYLE_PREDICT_LANE != dataPack->roadway.adjacentLeft_Lane.style)
    {
        paintLane(*dataPack, dataPack->roadway.adjacentLeft_Lane,
            smartImage, far_distance, adjacent_color, maskMode);
    }
   
    if (LDW_LANE_STYLE_NONE_LANE != dataPack->roadway.adjacentRightLane.style &&
        LDW_LANE_STYLE_PREDICT_LANE != dataPack->roadway.adjacentRightLane.style)
    {
        paintLane(*dataPack, dataPack->roadway.adjacentRightLane,
            smartImage, far_distance, adjacent_color, maskMode);
    }

    return true;
}

void RoadwayPainter::imageGrayToRGB(const unsigned char * gray, unsigned char * rgb, int _width, int _height)
{
    int length = _width * _height;
    unsigned char tc;
    for (int i = 0; i < length; i++)
    {
        tc = *(gray++);

        *(rgb++) = tc;
        *(rgb++) = tc;
        *(rgb++) = tc;
    }
}

void RoadwayPainter::outputParamFromMToMm(void * _roadwayParam)
{
    LdwDataPack *dataPack = (LdwDataPack *)_roadwayParam;

    dataPack->roadway.left_Lane.left_Boundary.coefficient[0] *= 1000.0f;
    dataPack->roadway.left_Lane.left_Boundary.coefficient[2] /= 1000.0f;
    dataPack->roadway.left_Lane.left_Boundary.coefficient[3] /= 1000000.0f;

    dataPack->roadway.left_Lane.rightBoundary.coefficient[0] *= 1000.0f;
    dataPack->roadway.left_Lane.rightBoundary.coefficient[2] /= 1000.0f;
    dataPack->roadway.left_Lane.rightBoundary.coefficient[3] /= 1000000.0f;


    dataPack->roadway.rightLane.left_Boundary.coefficient[0] *= 1000.0f;
    dataPack->roadway.rightLane.left_Boundary.coefficient[2] /= 1000.0f;
    dataPack->roadway.rightLane.left_Boundary.coefficient[3] /= 1000000.0f;

    dataPack->roadway.rightLane.rightBoundary.coefficient[0] *= 1000.0f;
    dataPack->roadway.rightLane.rightBoundary.coefficient[2] /= 1000.0f;
    dataPack->roadway.rightLane.rightBoundary.coefficient[3] /= 1000000.0f;


    dataPack->roadway.adjacentLeft_Lane.left_Boundary.coefficient[0] *= 1000.0f;
    dataPack->roadway.adjacentLeft_Lane.left_Boundary.coefficient[2] /= 1000.0f;
    dataPack->roadway.adjacentLeft_Lane.left_Boundary.coefficient[3] /= 1000000.0f;

    dataPack->roadway.adjacentLeft_Lane.rightBoundary.coefficient[0] *= 1000.0f;
    dataPack->roadway.adjacentLeft_Lane.rightBoundary.coefficient[2] /= 1000.0f;
    dataPack->roadway.adjacentLeft_Lane.rightBoundary.coefficient[3] /= 1000000.0f;

    dataPack->roadway.adjacentRightLane.left_Boundary.coefficient[0] *= 1000.0f;
    dataPack->roadway.adjacentRightLane.left_Boundary.coefficient[2] /= 1000.0f;
    dataPack->roadway.adjacentRightLane.left_Boundary.coefficient[3] /= 1000000.0f;

    dataPack->roadway.adjacentRightLane.rightBoundary.coefficient[0] *= 1000.0f;
    dataPack->roadway.adjacentRightLane.rightBoundary.coefficient[2] /= 1000.0f;
    dataPack->roadway.adjacentRightLane.rightBoundary.coefficient[3] /= 1000000.0f;
}

void RoadwayPainter::getBeginAndEndImageIndex(int &_bH, int &_eH, int _arrayIndex[], int closeH)
{
    for (int i = _bH; i < closeH; i++)
    {
        _bH = i;
        if (_arrayIndex[i] >= 0) break;
    }

    for (int i = _eH; i >= 0; i--)
    {
        _eH = i;
        if (_arrayIndex[i] >= 0) break;
    }
}

bool RoadwayPainter::paintLaneBoundary(SmartRgbImage &_img, int _bH, int _eH, int _lEdge[], int _rEdge[], int _alpha, int _color[], bool maskMode)
{
    int lx, rx;
    int R, G, B, alpha1;

    unsigned char *colData, *rowData;

    _alpha = (int)(((float)_alpha / 100.0f) * 128 + 0.5);
    alpha1 = 128 - _alpha;

    rowData = _img.mData + (_bH - 1) * _img.mStep;

    for (int h = _bH; h < _eH; h++)
    {
        rowData += _img.mStep;

        lx = _lEdge[h];
        rx = _rEdge[h];

        if (lx < 0 || rx < 0 || lx >= _img.mWidth || rx >= _img.mWidth)
            continue;

        colData = rowData + (lx - 1) * 3;

        for (int w = lx; w < rx; w++)
        {
            if (maskMode) {
                B = _color[0];
                G = _color[1];
                R = _color[2];
            }
            else {
                B = *(colData + 0);
                G = *(colData + 1);
                R = *(colData + 2);

                B = B * alpha1 + _color[0] * _alpha;
                G = G * alpha1 + _color[1] * _alpha;
                R = R * alpha1 + _color[2] * _alpha;

                R >>= 7;
                G >>= 7;
                B >>= 7;
            }

#ifdef WINDOWS_DEBUG
            *(colData++) = R;
            *(colData++) = G;
            *(colData++) = B;
#else
            *(colData++) = B;
            *(colData++) = G;
            *(colData++) = R;
#endif // WINDOWS_DEBUG
        }
    }
    return false;
}

bool RoadwayPainter::calculateImageIndex(void *_lens, float _worldX, float _worldY, float & _imageX, float & _imageY)
{
    LdwLensInfo *lens = (LdwLensInfo *)_lens;
    float x, y, z;
    float x0, y0, z0;

    x = _worldX;
    z = _worldY;
    y = lens->mountingHeight;

    x0 = lens->mCosRy * x + lens->mSinRy * z;
    y0 = lens->mSinRx * lens->mSinRy * x + lens->mCosRx * y - lens->mCosRy *lens->mSinRx * z;
    z0 = -lens->mCosRx * lens->mSinRy * x + lens->mSinRx * y + lens->mCosRx * lens->mCosRy * z;

    if (std::abs(z0) < FLT_MIN)
    {
        x0 = 0;
        y0 = 0;
    }
    else
    {

        x0 = x0 / z0;
        y0 = y0 / z0;
    }
    z0 = 1;

    float u0 = lens->xImageFocal;
    float v0 = lens->yImageFocal;

    _imageX = x0 * lens->xRatioFocalToPixel + u0;
    _imageY = y0 * lens->xRatioFocalToPixel + v0;
    return false;
}

void RoadwayPainter::getImageCurveIndex(void *_lens, void *_boundary, float _worldY,
    float & _imageX, float & _imageY)
{
    LdwLensInfo *lens = (LdwLensInfo *)_lens;
    LdwLaneBoundary *boundary = (LdwLaneBoundary *)_boundary;

    float worldX, y;

    worldX = 0.0f;
    y = 1.0f;

    for (int i = 0; i <= boundary->degree; i++)
    {
        worldX += boundary->coefficient[i] * y;
        y *= _worldY;
    }

    calculateImageIndex(lens, worldX, _worldY, _imageX, _imageY);
}

void RoadwayPainter::calculateCurveIndex(void *_lens, void *_boundary, SmartRgbImage &_image,
    float _by, float _ey, int _boundaryIndex[])
{
    LdwLensInfo *lens = (LdwLensInfo *)_lens;
    LdwLaneBoundary *boundary = (LdwLaneBoundary *)_boundary;

    float imgMx, imgMy, imgBx, imgBy, imgEx, imgEy;
    float my;
    int imgX, imgY;

    my = (_by + _ey) / 2;

    getImageCurveIndex(lens, boundary, _by, imgBx, imgBy);
    getImageCurveIndex(lens, boundary, _ey, imgEx, imgEy);
    getImageCurveIndex(lens, boundary, my, imgMx, imgMy);

    if (std::abs((int)imgBy - (int)imgEy) > 1)
    {
        imgX = (int)(imgMx + 0.5);
        imgY = (int)(imgMy + 0.5);

        imgX = std::max(0, std::min(imgX, _image.mWidth - 1));

        if (imgY >= 0 && imgY < _image.mHeight)
        {
            _boundaryIndex[imgY] = imgX;
        }

        if (imgY >= _image.mHeight)
        {
            calculateCurveIndex(lens, boundary, _image, my, _ey, _boundaryIndex);
        }
        else
        {
            calculateCurveIndex(lens, boundary, _image, _by, my, _boundaryIndex);
            calculateCurveIndex(lens, boundary, _image, my, _ey, _boundaryIndex);
        }
    }

    return;
}

void RoadwayPainter::doCurveInterpolation(int _bH, int _eH, int _curveEquation[])
{
    float by, bx, ey, ex, cy;
    int preH, endH;

    while (_curveEquation[_bH] < 0)
    {
        _curveEquation[_bH] = 0;

        _bH++;

        if (_bH >= _eH) break;
    }

    while (_curveEquation[_eH] < 0)
    {
        _curveEquation[_eH] = 0;

        _eH--;

        if (_bH >= _eH) break;
    }

    for (int h = _bH; h <= _eH; h++)
    {
        if (_curveEquation[h] >= 0)
        {
            preH = h;
        }
        else
        {
            for (int nh = h + 1; nh <= _eH; nh++)
            {
                if (_curveEquation[nh] >= 0)
                {
                    endH = nh;
                    break;
                }
            }

            by = (float)preH;
            bx = (float)_curveEquation[preH];

            ey = (float)endH;
            ex = (float)_curveEquation[endH];

            for (int nh = preH + 1; nh < endH; nh++)
            {
                cy = (float)nh;

                _curveEquation[nh] = (int)(((cy - by) * ex + (ey - cy) * bx) / (ey - by) + 0.5);
            }
        }
    }
}

void RoadwayPainter::getLineEquation(float _x1, float _y1, float _x2, float _y2, float & _k, float & _b)
{
    float y1y2;
    y1y2 = _y1 - _y2;

    if (std::abs(y1y2) < FLT_MIN)
    {
        _k = 0;
        _b = 0;
        return;
    }

    _k = (_x1 - _x2) / y1y2;
    _b = (_y1 * _x2 - _y2 * _x1) / y1y2;
}

void RoadwayPainter::drawLine(SmartRgbImage & _img, int _bh, int _eh, float _k, float _b, int _color[])
{
    unsigned char *imgData = _img.mData + _bh * _img.mStep - _img.mStep;

    for (int h = _bh; h < _eh; h++)
    {
        imgData += _img.mStep;

        int w = (int)(h * _k + _b);

        if (w < 1) continue;

        if (w >= _img.mWidth) continue;

        w = (w - 1) * 3;

        imgData[w + 0] = _color[0];
        imgData[w + 1] = _color[1];
        imgData[w + 2] = _color[2];

        w += 3;
        imgData[w + 0] = _color[0];
        imgData[w + 1] = _color[1];
        imgData[w + 2] = _color[2];

        w += 3;
        imgData[w + 0] = _color[0];
        imgData[w + 1] = _color[1];
        imgData[w + 2] = _color[2];
    }
}

RoadwayPainter::RoadwayPainter()
{

}

RoadwayPainter::~RoadwayPainter()
{

}

void RoadwayPainter::getImagePointFormWorldCoordinate(LdwDataPack & _dataPack,
    float _world_x, float _word_y, float & _img_x, float & _img_y)
{
    float x, y, z;
    float x0, y0, z0;

    x = _world_x;
    z = _word_y;
    y = _dataPack.lens.mountingHeight;

    x0 = _dataPack.lens.mCosRy * x + _dataPack.lens.mSinRy * z;
    y0 = _dataPack.lens.mSinRx * _dataPack.lens.mSinRy * x +
        _dataPack.lens.mCosRx * y -
        _dataPack.lens.mCosRy * _dataPack.lens.mSinRx * z;
    z0 = -_dataPack.lens.mCosRx * _dataPack.lens.mSinRy * x +
        _dataPack.lens.mSinRx * y +
        _dataPack.lens.mCosRx * _dataPack.lens.mCosRy * z;

    if (std::abs(z0) < FLT_MIN)
    {
        x0 = 0;
        y0 = 0;
    }
    else
    {
        x0 = x0 / z0;
        y0 = y0 / z0;
    }
    z0 = 1;

    float u0 = _dataPack.lens.xImageFocal;
    float v0 = _dataPack.lens.yImageFocal;

    _img_x = x0 * _dataPack.lens.xRatioFocalToPixel + u0;
    _img_y = y0 * _dataPack.lens.xRatioFocalToPixel + v0;
}

void RoadwayPainter::getConstParams(LdwDataPack & _dataPack, float & _alpha, float & _gamma)
{
    float img_x1, img_y1, img_x2, img_y2;
    float world_x1, world_y1, world_x2, world_y2;

    world_x1 = 0.0f;
    world_y1 = 1000.0f; //  1 meter

    getImagePointFormWorldCoordinate(_dataPack, world_x1, world_y1, img_x1, img_y1);

    world_x2 = 0.0f;
    world_y2 = 10000.0f;  // 10meter

    getImagePointFormWorldCoordinate(_dataPack, world_x2, world_y2, img_x2, img_y2);

    _alpha = (world_y1 * img_y1 - world_y2 * img_y2) / (world_y1 - world_y2);
    _gamma = (img_y1 - _alpha) * world_y1;
}

float RoadwayPainter::getEquationValue(int _degree, float _coefficient[], float _variate)
{
    float tValue = 1.0f;
    float rValue = 0.0f;

    for (int n = 0; n <= _degree; n++)
    {
        rValue += tValue * _coefficient[n];
        tValue *= _variate;
    }

    return rValue;
}

float RoadwayPainter::getEquationSlope(int _degree, float _coefficient[], float _variate)
{
    float tValue = 1.0f;
    float rValue = 0.0f;

    for (int n = 1; n <= _degree; n++)
    {
        rValue += ((float)n) * _coefficient[n] * tValue;
        tValue *= _variate;
    }

    return rValue;
}

void RoadwayPainter::getLaneMarkBoundary(LdwDataPack & _dataPack, float _far_distance,
    int _img_height, int _degree, float _coefficient[], int *_boundary)
{
    int boundary_x, boundary_y;
    float world_x, world_y, img_x, img_y;

    float alpha, gamma;

    getConstParams(_dataPack, alpha, gamma);

    float maxImageHeight = (float)(_img_height - alpha + 10);
    float imageHeight, yValue, xValue, kValue;

    imageHeight = maxImageHeight + 1.0f;

    while (true)
    {
        imageHeight -= 1.0f;
        yValue = gamma / imageHeight;

        if (yValue > _far_distance || imageHeight < 0) break;

        xValue = getEquationValue(_degree, _coefficient, yValue);
        kValue = getEquationSlope(_degree, _coefficient, yValue);

        world_x = xValue;
        world_y = yValue;

        getImagePointFormWorldCoordinate(_dataPack, world_x, world_y, img_x, img_y);

        boundary_x = (int)(img_x + 0.5f);
        boundary_y = (int)(img_y + 0.5f) - 1;

        if (boundary_y > _img_height - 1)
            boundary_y = _img_height - 1;

        _boundary[boundary_y] = boundary_x;
    }
}

float RoadwayPainter::getFarDistance(LdwDataPack & _dataPack)
{
    const float m_to_mm = 1000.f;
    const float const_curve_rate = 1 / (2000.f * m_to_mm);
    float curvature;
    float max_curve_rate = _dataPack.roadway.left_Lane.rightBoundary.coefficient[2];

    max_curve_rate = std::abs(max_curve_rate);
    max_curve_rate = std::max(max_curve_rate,
        std::abs(_dataPack.roadway.rightLane.left_Boundary.coefficient[2]));

    if (max_curve_rate < const_curve_rate)
        max_curve_rate = const_curve_rate;
    
    curvature = 0.5f / max_curve_rate;

    if (curvature >= 500.f * m_to_mm)
        return 50 * m_to_mm;
    else if (curvature > 250.f * m_to_mm && curvature < 500.f * m_to_mm)
        return 40 * m_to_mm; 
    else
        return 30 * m_to_mm;
}

void RoadwayPainter::paintLane(LdwDataPack & _dataPack, LdwLane & _lane,
    SmartRgbImage & _smart_image, float _far_distance, int _color[],
    bool _mask_mode)
{
    const float meter5 = 5000.f;

    static int *left_boundary = new int[_smart_image.mHeight];
    static int *right_boundary= new int[_smart_image.mHeight];

    memset(left_boundary, -1, _smart_image.mHeight * sizeof(int));
    memset(right_boundary, -1, _smart_image.mHeight * sizeof(int));

    int bH, eH, by, ey;
    float img_x, img_y;

    getLaneMarkBoundary(_dataPack, _far_distance, _smart_image.mHeight,
        _lane.left_Boundary.degree, _lane.left_Boundary.coefficient, left_boundary);

    getLaneMarkBoundary(_dataPack, _far_distance, _smart_image.mHeight,
        _lane.rightBoundary.degree, _lane.rightBoundary.coefficient, right_boundary);

    bH = 0;
    eH = _smart_image.mHeight - 1;

    getBeginAndEndImageIndex(bH, eH, left_boundary, _smart_image.mHeight);
    getBeginAndEndImageIndex(bH, eH, right_boundary, _smart_image.mHeight);

    doCurveInterpolation(bH, eH, left_boundary);
    doCurveInterpolation(bH, eH, right_boundary);

    if (LDW_LANE_STYLE_BROKEN_LANE == _lane.style ||
        LDW_LANE_STYLE_DOUBLE_BROKEN_LANE == _lane.style)
    {
        for (float d = meter5; d < _far_distance; d += meter5 * 2)
        {

            getImagePointFormWorldCoordinate(_dataPack, 0, d, img_x, img_y);
            ey = (int)(img_y + 0.5f);

            getImagePointFormWorldCoordinate(_dataPack, 0, d + meter5, img_x, img_y);
            by = (int)(img_y + 0.5f);

            if (ey - by < 0) continue;

            by = std::max(bH, by);
            ey = std::min(eH, ey);

            if (ey - by < 0) continue;

            paintLaneBoundary(_smart_image, by, ey,
                left_boundary, right_boundary, 80, _color, _mask_mode);
        }
    }
    else if (LDW_LANE_STYLE_SOLID_LANE == _lane.style ||
        LDW_LANE_STYLE_DOUBLE_SOLID_LANE == _lane.style ||
        LDW_LANE_STYLE_TRIPLE_LANE == _lane.style)
    {
        paintLaneBoundary(_smart_image, bH, eH,
            left_boundary, right_boundary, 80, _color, _mask_mode);
    }
}

void RoadwayPainter::drawWhenManualLearnMode(LdwDataPack &_dataPack,
    SmartRgbImage &_smartImage, int _color[])
{
    float k1, k2, b1, b2;
    int bh;

    b1 = _dataPack.roadway.left_Lane.left_Boundary.coefficient[0];
    k1 = _dataPack.roadway.left_Lane.left_Boundary.coefficient[1];

    b2 = _dataPack.roadway.rightLane.left_Boundary.coefficient[0];
    k2 = _dataPack.roadway.rightLane.left_Boundary.coefficient[1];

    if (std::abs(k1 - k2) < FLT_MIN) bh = (int)(3 * _smartImage.mHeight / 4);
    else bh = (int)((b2 - b1) / (k1 - k2));

    drawLine(_smartImage, bh, _smartImage.mHeight,
        _dataPack.roadway.left_Lane.left_Boundary.coefficient[1],
        _dataPack.roadway.left_Lane.left_Boundary.coefficient[0],
        _color);

    drawLine(_smartImage, bh, _smartImage.mHeight,
        _dataPack.roadway.left_Lane.rightBoundary.coefficient[1],
        _dataPack.roadway.left_Lane.rightBoundary.coefficient[0],
        _color);

    drawLine(_smartImage, bh, _smartImage.mHeight,
        _dataPack.roadway.rightLane.left_Boundary.coefficient[1],
        _dataPack.roadway.rightLane.left_Boundary.coefficient[0],
        _color);

    drawLine(_smartImage, bh, _smartImage.mHeight,
        _dataPack.roadway.rightLane.rightBoundary.coefficient[1],
        _dataPack.roadway.rightLane.rightBoundary.coefficient[0],
        _color);
}

