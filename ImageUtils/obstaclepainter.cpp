#include "obstaclepainter.h"
#include "smarterFont.h"
#include "obstacleData.h"
#include <algorithm>
#include <stdio.h>

ObstaclePainter::ObstaclePainter()
{

}

ObstaclePainter::~ObstaclePainter()
{

}

void ObstaclePainter::paintRectBorder(unsigned char *_imageData, OutputObstacles &_rectBorder,
                                      int _width, int _height, bool isWarn)
{
    int rectBx, rectEx, rectBy, rectEy, rectWidth_, rectHeight;

    rectBx = _rectBorder.firstPointX;
    rectBx = std::min(rectBx, (int)_rectBorder.secondPointX);
    rectBx = std::min(rectBx, (int)_rectBorder.thirdPointX);
    rectBx = std::min(rectBx, (int)_rectBorder.fourthPointX);

    rectEx = _rectBorder.firstPointX;
    rectEx = std::max(rectEx, (int)_rectBorder.secondPointX);
    rectEx = std::max(rectEx, (int)_rectBorder.thirdPointX);
    rectEx = std::max(rectEx, (int)_rectBorder.fourthPointX);

    rectBy = _rectBorder.firstPointY;
    rectBy = std::min(rectBy, (int)_rectBorder.secondPointY);
    rectBy = std::min(rectBy, (int)_rectBorder.thirdPointY);
    rectBy = std::min(rectBy, (int)_rectBorder.fourthPointY);

    rectEy = _rectBorder.firstPointY;
    rectEy = std::max(rectEy, (int)_rectBorder.secondPointY);
    rectEy = std::max(rectEy, (int)_rectBorder.thirdPointY);
    rectEy = std::max(rectEy, (int)_rectBorder.fourthPointY);

    rectWidth_ = rectEx - rectBx;
    rectHeight = rectEy - rectBy;

    if (rectBx < 0 || rectBy < 0 || rectEx >= _width|| rectEy >= _height||rectWidth_==0||rectHeight==0)
        return;

    unsigned char *pData, *pLineData;

    int bx[8], ex[8], by[8], ey[8];

    int lineLength, minBorderLength, lineWidth;

    int imgStep = _width * 3;

    int r, g, b, tr, tg, tb;

    // static int showLineN = 0;

    // showLineN += 1;

    // if (showLineN > (1 << 20))
    // {
        // showLineN = 0;
    // }

    if (isWarn)
    {
        tr = 255;
        tg = 22;
        tb = 22;
    }
    else
    {
        tr = 22;
        tg = 255;
        tb = 22;
    }

    minBorderLength = std::min(rectWidth_, rectHeight);

    static const int minLineLength = 14;

    if (minBorderLength <= minLineLength)
    {
        lineLength = minBorderLength / 2;
    }
    else
    {
        lineLength = (minBorderLength + 9) / 10;

        lineLength = std::max(minLineLength / 2, lineLength);
    }

    static const int minLineWidth = 64;

    if (minBorderLength <= minLineWidth)
    {
        lineWidth = 2;
    }
    else
    {
        lineWidth = (minBorderLength + 63) >> 6;
        lineWidth = std::max(lineWidth, 2);
    }

    bx[0] = rectBx;
    ex[0] = bx[0] + lineLength;
    by[0] = rectBy;
    ey[0] = by[0] + lineWidth;

    bx[1] = rectBx;
    ex[1] = bx[1] + lineWidth;
    by[1] = rectBy;
    ey[1] = by[1] + lineLength;

    bx[2] = rectBx + rectWidth_ - lineLength;
    ex[2] = bx[2] + lineLength;
    by[2] = rectBy;
    ey[2] = by[2] + lineWidth;

    bx[3] = rectBx + rectWidth_ - lineWidth;
    ex[3] = bx[3] + lineWidth;
    by[3] = rectBy;
    ey[3] = by[3] + lineLength;

    bx[4] = rectBx;
    ex[4] = bx[4] + lineLength;
    by[4] = rectBy + rectHeight - lineWidth;
    ey[4] = by[4] + lineWidth;

    bx[5] = rectBx;
    ex[5] = bx[5] + lineWidth;
    by[5] = rectBy + rectHeight - lineLength;
    ey[5] = by[5] + lineLength;

    bx[6] = rectBx + rectWidth_ - lineLength;
    ex[6] = bx[6] + lineLength;
    by[6] = rectBy + rectHeight - lineWidth;
    ey[6] = by[6] + lineWidth;

    bx[7] = rectBx + rectWidth_ - lineWidth;
    ex[7] = bx[7] + lineWidth;
    by[7] = rectBy + rectHeight - lineLength;
    ey[7] = by[7] + lineLength;

    static const int radio = (int)(0.3 * 1024 + 0.5);
    pLineData = _imageData + rectBy * imgStep;

    for (int h = 0; h < rectHeight; h++)
    {
        pData = pLineData + rectBx * 3;
        for (int w = 0; w < rectWidth_; w++)
        {
            b = *(pData);
            g = *(pData + 1);
            r = *(pData + 2);

            r = tr * radio + r * (1024 - radio);
            g = tg * radio + g * (1024 - radio);
            b = tb * radio + b * (1024 - radio);

            r >>= 10;
            g >>= 10;
            b >>= 10;

#ifdef _CONSOLE
            *(pData++) = b;
            *(pData++) = g;
            *(pData++) = r;
#else
            *(pData++) = r;
            *(pData++) = g;
            *(pData++) = b;
#endif // _CONSOLE

        }
        pLineData += imgStep;
    }

    for (int i = 0; i < 8; i++)
    {
        bx[i] = std::max(0, bx[i]);
        ex[i] = std::min(ex[i], _width);
        by[i] = std::max(0, by[i]);
        ey[i] = std::min(ey[i], _height);

        for (int h = by[i]; h < ey[i]; h++)
        {
            for (int w = bx[i]; w < ex[i]; w++)
            {
                *(_imageData + h * imgStep + 3 * w + 0) = tr;
                *(_imageData + h * imgStep + 3 * w + 1) = tg;
                *(_imageData + h * imgStep + 3 * w + 2) = tb;
            }
        }
    }

    // int beginLine = showLineN % rectHeight + rectBy;

    // beginLine = std::max(3 + rectBy, std::min(rectHeight + rectBy - 7, beginLine));

    // pLineData = _imageData + beginLine * imgStep;

    // for (int h = 0; h < 3; h++)
    // {
        // pData = pLineData + (rectBx + 10) * 3;
        // for (int w = 0; w < rectWidth_ - 20; w++)
        // {
            // *(pData++) = tr;
            // *(pData++) = tg;
            // *(pData++) = tb;
        // }
        // pLineData += imgStep;
    // }
}

bool ObstaclePainter::paintObstacle(void * _obstacleParam, unsigned char * _rgbImageData,
                                    int width, int height, bool showDetials, bool singleObs)
{
    const int offsetX = 0;
    const int offsetY = 0;

    int blockNum = reinterpret_cast<int*>(_obstacleParam)[0];
    if(blockNum > 0) {
        OutputObstacles *pOutputObstacles = reinterpret_cast<OutputObstacles*> (reinterpret_cast<int*>(_obstacleParam) + 2);

        for(int i = 0; i < blockNum; i++) {
            if (singleObs) {
                if (pOutputObstacles[i].stateLabel == 1) {
                    drawObsFrame(_rgbImageData, pOutputObstacles[i], width, height, offsetX, offsetY, showDetials);
                    break;
                }
            } else {
                drawObsFrame(_rgbImageData, pOutputObstacles[i], width, height, offsetX, offsetY, showDetials);
            }
        }
    }

    return true;
}

void ObstaclePainter::drawObsInfo(unsigned char *_imageData, OutputObstacles &_rectBorder,
                                  int _width, int _height,const unsigned char color[3])
{
    char TTC_time[50] = "";
    char TTC_dist[50] = "";
    char real3DLeftX[50] = "";
    char real3DRightX[50] = "";
    char real3DCenterX[50] = "";
    char real3DHeight[50] = "";
    char TTC_speed[50] = "";
    char TTC_speed_X[50] = "";
    char TTC_WidthX[50] = "";
    char TTC_Collasion_X[50] = "";

    sprintf(TTC_time, "TTCZ: %.2fs", _rectBorder.fuzzyCollisionTimeZ);
    sprintf(TTC_dist, "DistZ: %.2fm", _rectBorder.fuzzyRelativeDistanceZ);
    sprintf(real3DLeftX, "LeftX: %.2fm", _rectBorder.fuzzy3DLeftX);
    sprintf(real3DRightX, "RightX: %.2fm", _rectBorder.fuzzy3DRightX);
    sprintf(real3DCenterX, "CenterX: %.2fm", _rectBorder.fuzzy3DCenterX);
    sprintf(real3DHeight, "Height: %.2fm", (_rectBorder.real3DUpY - _rectBorder.real3DLowY));
    sprintf(TTC_speed, "SpeedZ: %f", _rectBorder.fuzzyRelativeSpeedZ);
    sprintf(TTC_speed_X, "SpeedX: %f m/s", _rectBorder.fuzzyRelativeSpeedCenterX);
    sprintf(TTC_WidthX, "WidthX: %.2fm", _rectBorder.fuzzy3DWidth);
    sprintf(TTC_Collasion_X, "CollasionX: %d", _rectBorder.fuzzyCollisionX);

    int rectLeftTopPoint[2];
    rectLeftTopPoint[0] = _rectBorder.firstPointX;
    rectLeftTopPoint[1] = _rectBorder.firstPointY;
    int org[2];
    org[0] = rectLeftTopPoint[0] + 3;
    org[1] = rectLeftTopPoint[1] + 18;
    smarterFont::putText(_imageData,_width,_height,TTC_time,org,3,0.6,color,1);
    org[0] = rectLeftTopPoint[0] + 3;
    org[1] = rectLeftTopPoint[1] + 36;
    smarterFont::putText(_imageData,_width,_height,TTC_dist,org,3,0.6,color,1);
    org[0] = rectLeftTopPoint[0] + 3;
    org[1] = rectLeftTopPoint[1] + 54;
    smarterFont::putText(_imageData,_width,_height,real3DLeftX,org,3,0.6,color,1);
    org[0] = rectLeftTopPoint[0] + 3;
    org[1] = rectLeftTopPoint[1] + 72;
    smarterFont::putText(_imageData,_width,_height,real3DRightX,org,3,0.6,color,1);
    org[0] = rectLeftTopPoint[0] + 3;
    org[1] = rectLeftTopPoint[1] + 90;
    smarterFont::putText(_imageData,_width,_height,real3DCenterX,org,3,0.6,color,1);
    org[0] = rectLeftTopPoint[0] + 3;
    org[1] = rectLeftTopPoint[1] + 108;
    smarterFont::putText(_imageData,_width,_height,real3DHeight,org,3,0.6,color,1);
    org[0] = rectLeftTopPoint[0] + 3;
    org[1] = rectLeftTopPoint[1] + 126;
    smarterFont::putText(_imageData,_width,_height,TTC_speed,org,3,0.6,color,1);
    org[0] = rectLeftTopPoint[0] + 3;
    org[1] = rectLeftTopPoint[1] + 144;
    smarterFont::putText(_imageData,_width,_height,TTC_speed_X,org,3,0.6,color,1);
    org[0] = rectLeftTopPoint[0] + 3;
    org[1] = rectLeftTopPoint[1] + 162;
    smarterFont::putText(_imageData,_width,_height,TTC_WidthX,org,3,0.6,color,1);
    org[0] = rectLeftTopPoint[0] + 3;
    org[1] = rectLeftTopPoint[1] + 180;
    smarterFont::putText(_imageData,_width,_height,TTC_Collasion_X,org,3,0.6,color,1);
}

void ObstaclePainter::drawObsFrame(unsigned char *_imageData, OutputObstacles &_rectBorder,
                                   int _width, int _height,int _offsetX, int _offsetY, bool showDetails)
{
    const unsigned char colorPink[3]={255, 0, 255};
    const unsigned char colorRed[3]={255, 0, 0};
    const unsigned char colorGreen[3]={0, 255, 0};
    const unsigned char colorYellow[3]={255, 255, 0};

    static const float alarmTh = 2700;
    _rectBorder.firstPointX += _offsetX;
    _rectBorder.firstPointY += _offsetY;
    _rectBorder.secondPointX += _offsetX;
    _rectBorder.secondPointY += _offsetY;
    _rectBorder.thirdPointX += _offsetX;
    _rectBorder.thirdPointY += _offsetY;
    _rectBorder.fourthPointX += _offsetX;
    _rectBorder.fourthPointY += _offsetY;

    int org[2];
    char carSpeed[50] = "";
    org[0] = 10;
    org[1] = 35;
    sprintf(carSpeed, "car speed: %3f", _rectBorder.currentSpeed);
    smarterFont::putText(_imageData,_width,_height,carSpeed,org,3,0.625,colorRed,1);

    char curFrameRate[50] = "";
    org[0] = 10;
    org[1] = 50;
    sprintf(curFrameRate, "frame rate: %3f", _rectBorder.frameRate);
    smarterFont::putText(_imageData,_width,_height,curFrameRate,org,3,0.625,colorRed,1);

    float distance = _rectBorder.avgDistanceZ; // write average distance

    switch(_rectBorder.classLabel) {
    case 4:
    case 5:
    {
        // continuous obstacle, pink color label
        // draw continuous obstacle boundary in gray image
        smarterPoint p0(_rectBorder.firstPointX << 16,_rectBorder.firstPointY << 16);
        smarterPoint p1(_rectBorder.secondPointX << 16,_rectBorder.secondPointY << 16);
        smarterPoint p2(_rectBorder.thirdPointX << 16,_rectBorder.thirdPointY << 16);
        smarterPoint p3(_rectBorder.fourthPointX << 16,_rectBorder.fourthPointY << 16);
        ThickLine(_imageData,_width,_height,p0,p1,colorPink,2,0,3);
        ThickLine(_imageData,_width,_height,p1,p2,colorPink,2,0,3);
        ThickLine(_imageData,_width,_height,p2,p3,colorPink,2,0,3);
        ThickLine(_imageData,_width,_height,p3,p0,colorPink,2,0,3);

        char T[50] = "";

        if ((_rectBorder.classLabel == 4)) // left continuous obstacle
        {
            sprintf(T, "%.2fm", _rectBorder.nearDistanceZ); // continuous obstacle, write near end distance information
            org[0] = _rectBorder.secondPointX + 10;
            org[1] = _rectBorder.secondPointY + 25;
            smarterFont::putText(_imageData,_width,_height,T,org,3,1,colorPink,1);

            sprintf(T, "%.2fm", _rectBorder.farDistanceZ); // continuous obstacle, write far end distance information
            org[0] = _rectBorder.thirdPointX - 70;
            org[1] = _rectBorder.thirdPointY + 20;
            smarterFont::putText(_imageData,_width,_height,T,org,3,1,colorPink,1);
        }
        else if ((_rectBorder.classLabel == 5)) // right continuous obstacle
        {
            sprintf(T, "%.2fm", _rectBorder.nearDistanceZ); // continuous obstacle, write near end distance information
            org[0] = _rectBorder.secondPointX - 70;
            org[1] = _rectBorder.secondPointY + 25;
            smarterFont::putText(_imageData,_width,_height,T,org,3,1,colorPink,1);

            sprintf(T, "%.2fm", _rectBorder.farDistanceZ); // continuous obstacle, write far end distance information
            org[0] = _rectBorder.thirdPointX + 10;
            org[1] = _rectBorder.thirdPointY + 20;
            smarterFont::putText(_imageData,_width,_height,T,org,3,1,colorPink,1);
        }
    }
        break;
    case 6:
    {
        if(_rectBorder.stateLabel == 1) // nearest obstacle in warning area
        {
            // add for TTC information display
            if (_rectBorder.fuzzyEstimationValid == 1)
            {
                if ((_rectBorder.fuzzyCollisionTimeZ <= alarmTh)
                        &&(_rectBorder.fuzzyCollisionTimeZ > 0))
                {

                }
                if (showDetails) {
//                    drawObsInfo(_imageData,_rectBorder,_width,_height,colorRed);
                }
            }

            char T[50] = "";
            org[0] = _rectBorder.firstPointX;
            org[1] = _rectBorder.firstPointY -3;
            sprintf(T, "Z: %.2fm", distance);
//            smarterFont::putText(_imageData,_width,_height,T,org,3,0.8,colorRed,1);
//            putText(imgRGB, T, Point(rt.x, rt.y - 3), 2, 0.5, colorRed, 1);
        }
        else if(_rectBorder.stateLabel == 2) // other obstacles in warning area
        {
            // add for TTC information display
            if (_rectBorder.fuzzyEstimationValid == 1)
            {
                if ((_rectBorder.fuzzyCollisionTimeZ <= alarmTh)
                        &&(_rectBorder.fuzzyCollisionTimeZ > 0))
                {

                }
                if (showDetails) {
//                    drawObsInfo(_imageData,_rectBorder,_width,_height,colorYellow);
                }
            }

            char T[50] = "";
            org[0] = _rectBorder.firstPointX;
            org[1] = _rectBorder.firstPointY -3;
            sprintf(T, "Z: %.2fm", distance);
//            smarterFont::putText(_imageData,_width,_height,T,org,3,0.8,colorYellow,1);
        }
        else if(_rectBorder.stateLabel == 3) // obstacles out of warning area
        {
            // add for TTC information display
            if (_rectBorder.fuzzyEstimationValid == 1)
            {
                if ((_rectBorder.fuzzyCollisionTimeZ <= alarmTh)
                        &&(_rectBorder.fuzzyCollisionTimeZ > 0))
                {
                }
                if (showDetails) {
//                    drawObsInfo(_imageData,_rectBorder,_width,_height,colorGreen);
                }
            }

            char T[50] = "";
            org[0] = _rectBorder.firstPointX;
            org[1] = _rectBorder.firstPointY -3;
            sprintf(T, "Z: %.2fm", distance);
//            smarterFont::putText(_imageData,_width,_height,T,org,3,0.8,colorGreen,1);
        }
    }
        break;
    default:
    {
        if(_rectBorder.stateLabel == 1) // nearest obstacle in warning area
        {
            paintRectBorder(_imageData,_rectBorder,_width,_height,true);

            // add for TTC information display
            if (_rectBorder.fuzzyEstimationValid == 1)
            {
                if ((_rectBorder.fuzzyCollisionTimeZ <= alarmTh)
                        &&(_rectBorder.fuzzyCollisionTimeZ > 0))
                {

                }
                if (showDetails) {
                    drawObsInfo(_imageData,_rectBorder,_width,_height,colorRed);
                }
            }

            char T[50] = "";
            org[0] = _rectBorder.firstPointX;
            org[1] = _rectBorder.firstPointY -3;
            sprintf(T, "Z: %.2fm", distance);
            smarterFont::putText(_imageData,_width,_height,T,org,3,0.8,colorRed,1);
        }
        else if(_rectBorder.stateLabel == 2) // other obstacles in warning area
        {
            paintRectBorder(_imageData,_rectBorder,_width,_height,true);

            // add for TTC information display
            if (_rectBorder.fuzzyEstimationValid == 1)
            {
                if ((_rectBorder.fuzzyCollisionTimeZ <= alarmTh)
                        &&(_rectBorder.fuzzyCollisionTimeZ > 0))
                {

                }
                if (showDetails) {
                    drawObsInfo(_imageData,_rectBorder,_width,_height,colorYellow);
                }
            }

            char T[50] = "";
            org[0] = _rectBorder.firstPointX;
            org[1] = _rectBorder.firstPointY -3;
            sprintf(T, "Z: %.2fm", distance);
            smarterFont::putText(_imageData,_width,_height,T,org,3,0.8,colorYellow,1);
        }
        else if(_rectBorder.stateLabel == 3) // obstacles out of warning area
        {
            paintRectBorder(_imageData,_rectBorder,_width,_height,false);

            // add for TTC information display
            if (_rectBorder.fuzzyEstimationValid == 1)
            {
                if ((_rectBorder.fuzzyCollisionTimeZ <= alarmTh)
                        &&(_rectBorder.fuzzyCollisionTimeZ > 0))
                {

                }
                if (showDetails) {
                    drawObsInfo(_imageData,_rectBorder,_width,_height,colorGreen);
                }
            }

            char T[50] = "";
            org[0] = _rectBorder.firstPointX;
            org[1] = _rectBorder.firstPointY -3;
            sprintf(T, "Z: %.2fm", distance);
            smarterFont::putText(_imageData,_width,_height,T,org,3,0.8,colorGreen,1);
        }

        char TT[50] = "";
        org[0] = _rectBorder.firstPointX;
        org[1] = _rectBorder.firstPointY -22;
        sprintf(TT, "Id: %d", _rectBorder.trackId); // track buffer id !!!
        smarterFont::putText(_imageData,_width,_height,TT,org,3,0.5,colorYellow,1);
    }
        break;
    }
}
