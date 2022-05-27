#include <algorithm>
#include <vector>
#include <list>
#include <ctime>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <numeric>
#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "disparityconvertor.h"
#include "frameformat.h"

using namespace std;

static const int kMaxDisparityValue = 81;

DisparityConvertor::DisparityConvertor()
{

}

void DisparityConvertor::convertDisparity2FloatFormat(const unsigned char* src, int width, int height, int bitNum, float* dest)
{
    assert(src);

    for (int i = 0; i < height; i++) {
        float *data = dest + i*width;
        for (int j = 0; j < width; j++) {
            switch (bitNum) {
            case 5:
            {
                data[j] = (*((short*)src) & 0x0FFF) / 32.0f;
                src += 2;
            }
                break;
            case 8: //todo
            {

            }
                break;
            default:
                break;
            }
        }
    }
}

float DisparityConvertor::getPointDisparityValue(const unsigned char *src, int width, int height, int bitNum, int posX, int posY)
{
    float disparityFloat = 0.0f;
    switch (bitNum) {
    case 5:
    {
        disparityFloat = ((short)(*((short*)src+ (posY * width + posX))) & 0x0FFF) / 32.0f;
        break;
    }
    case 8: //todo
        break;
    default:
        break;
    }

    return disparityFloat;
}

void DisparityConvertor::getPointXYZDistance(const unsigned char *src, int width, int height, int bitNum,
                                             float baseline, float focus, int cx, int cy,
                                             int posX, int posY, float &xDistance, float &yDistance, float &zDistance)
{
    assert(src);

    float disparity = getPointDisparityValue(src, width, height, bitNum, posX, posY);

    if(fabs(disparity) < 1e-6) {
        xDistance = 10000000.0f;
        yDistance = 10000000.0f;
        zDistance = 10000000.0f;
        return;
    }

    float bf = (float)(baseline * focus);
    xDistance = (float)((baseline * (posX - cx)) / disparity);
    yDistance = (float)((baseline * (posY - cy)) / disparity);
    zDistance = (float)(bf / disparity);
}

void DisparityConvertor::getRectZDistance(const unsigned char *src, int width, int height, int bitNum,
                                          float baseline, float focus,
                                          int posX0, int posY0, int posX1, int posY1, float &zDistance)
{
    int pointCount = 0;
    float meanDisparity = 0.0f;
    float medianDisparity = 0.0f;
    float subOfMeanAndMedianThreshold = 1.0f;
    float varianceDisparity = 0.0f;
    float varianceThreshold = 0.2f;
    int minNum;
    int maxNum;
    float sumOfDisparity = 0.0f;
    float sumOfVariance = 0.0f;

    static float *rectDisparityValue = new float[width * height];
    memset(rectDisparityValue, 0, (width * height) * sizeof(float));

    for (int y = posY0; y < posY1; y++) {
        for (int x = posX0; x < posX1; x++) {
            float pointDisparityValue = getPointDisparityValue(src, width, height, bitNum, x, y);
            if (pointDisparityValue > 0) {
                rectDisparityValue[pointCount] = pointDisparityValue;
                pointCount++;
            }
        }
    }
    if (pointCount == 0) {
        zDistance= 0;
        return;
    } else {
        std::sort(rectDisparityValue, rectDisparityValue + pointCount -1);

        medianDisparity = rectDisparityValue[(int)(pointCount / 2.0f - 0.5)];

        minNum = pointCount * 0.1;
        maxNum = pointCount * 0.9;
        for (int m = minNum; m < maxNum; m++) {
            sumOfDisparity += rectDisparityValue[m];
        }
        meanDisparity = ((double)sumOfDisparity / (maxNum - minNum));

        for (int m = minNum; m < maxNum; m++) {
            sumOfVariance = sumOfVariance + (rectDisparityValue[m]- meanDisparity)*(rectDisparityValue[m] - meanDisparity);
        }

        varianceDisparity = sqrt(sumOfVariance / (maxNum - minNum));

        if ((fabs(meanDisparity - medianDisparity) > subOfMeanAndMedianThreshold)
                || (varianceDisparity > varianceThreshold)) {
            zDistance = 0;
        } else {
            zDistance = (baseline * focus) / (medianDisparity);
        }
    }
}

void DisparityConvertor::getWholeXDistance(const float* disparity, int width, int height, float baseline, int cx, float* xDistance)
{
    assert(disparity && xDistance);

    float* tmpdistanceX = xDistance;
    const float* tmpDisparity = disparity;
    int length = width * height;
    float scale = (float)width / 1280.f;

    int X = 0;
    for (int i = 0; i < length; i++) {
        if(fabs(*tmpDisparity) < 1e-6) {
            *tmpdistanceX++ = 100000000.0f;
            tmpDisparity++;
            continue;
        }
        X = i % width;
        *tmpdistanceX++ = (float)((baseline * (X - scale * cx)) / (scale * (*tmpDisparity++)));
    }
}

void DisparityConvertor::getWholeYDistance(const float* disparity, int width, int height, float baseline, int cy, float* yDistance)
{
    assert(disparity && yDistance);

    float* tmpdistanceY = yDistance;
    const float* tmpDisparity = disparity;
    int length = width * height;
    float scale = (float)height / 1280.f;

    int Y = 0;
    for (int i = 0; i < length; i++) {
        if(fabs(*tmpDisparity) < 1e-6) {
            *tmpdistanceY++ = 100000000.0f;
            tmpDisparity++;
            continue;
        }
        Y = i / width;
        *tmpdistanceY++ = (float)((baseline * (Y - scale * cy)) / (scale * (*tmpDisparity++)));
    }
}

void DisparityConvertor::getWholeZDistance(const float* disparity, int width, int height, float baseline, float focus, float* zDistance)
{
    assert(disparity && zDistance);

    float bf = (float)(baseline * focus);
    int length = width * height;

    float* tmpdistanceZ = zDistance;
    const float* tmpDisparity = disparity;
    for (int i = 0; i < length; i++) {
        if(fabs(*tmpDisparity) < 1e-6) {
            *tmpdistanceZ++ = 100000000.0f;
            tmpDisparity++;
            continue;
        }
        *tmpdistanceZ++ = bf / (*tmpDisparity++);
    }
}

void DisparityConvertor::generateLookUpTableX(int width, int bitNum, float baseline, int cx, float* lookUpTableX)
{
    assert(lookUpTableX);

    float* lutPtr = lookUpTableX;
    int floatNum = pow(2, bitNum);

    for (int x = 0; x < width; x++) {
        *lutPtr++ = 1000000.0f;
        for (int i = 1; i < kMaxDisparityValue * floatNum; i++) {
            *lutPtr++ = (float)((baseline * (x - cx)) / (i / (float)floatNum));
        }
    }
}

void DisparityConvertor::generateLookUpTableY(int height, int bitNum, float baseline, int cy, float* lookUpTableY)
{
    assert(lookUpTableY);

    float* lutPtr = lookUpTableY;
    int floatNum = pow(2, bitNum);

    for (int y = 0; y < height; y++) {
        *lutPtr++ = 1000000.0f;
        for (int i = 1; i < kMaxDisparityValue * floatNum; i++) {
            *lutPtr++ = (float)((baseline * (y - cy)) / (i / (float)floatNum));
        }
    }
}

void DisparityConvertor::generateLookUpTableZ(int bitNum, float baseline, float focus, float* lookUpTableZ)
{
    assert(lookUpTableZ);

    float bf = (float)(baseline * focus);
    float* lutPtr = lookUpTableZ;
    int floatNum = pow(2, bitNum);

    *lutPtr++ = 100.0f;
    for (int i = 1; i < kMaxDisparityValue * floatNum; i++) {
        *lutPtr++ = bf / (i / (float)floatNum);
    }
}

void DisparityConvertor::getWholeXDistanceByLookupTable(const unsigned char* src, int width, int height, int bitNum, float* lookUpTableX, float* distanceX)
{
    assert(src && lookUpTableX && distanceX);

    float* floatPointer = (float*)(distanceX);
    float* floatLookUpTableBufferBase = lookUpTableX;
    int floatNum = pow(2, bitNum);
    int tmpX = 0;

    for (int i = 0; i < width * height; i++) {
        tmpX = i%width;
        *(floatPointer++) = *(float*)(floatLookUpTableBufferBase + tmpX*kMaxDisparityValue*floatNum + ((*(short*)src)&0x0fff));
        src += 2;
    }
}

void DisparityConvertor::getWholeYDistanceByLookupTable(const unsigned char* src, int width, int height, int bitNum, float* lookUpTableY, float* distanceY)
{
    assert(src && lookUpTableY && distanceY);

    float* floatPointer = (float*)(distanceY);
    float* floatLookUpTableBufferBase = lookUpTableY;
    int floatNum = pow(2, bitNum);
    int tmpY = 0;

    for (int i = 0; i < width * height; i++) {
        tmpY = i/width;
        *floatPointer++ = *(float*)(floatLookUpTableBufferBase + tmpY*kMaxDisparityValue*floatNum + ((*(short*)src)&0x0fff));
        src += 2;
    }
}

void DisparityConvertor::getWholeZDistanceByLookupTable(const unsigned char* src, int width, int height, float* lookUpTableZ, float* distanceZ)
{
    assert(src && lookUpTableZ && distanceZ);

    float* floatPointer = (float*)(distanceZ);
    float* floatLookUpTableBufferBase = lookUpTableZ;
    for (int i = 0; i < width * height; i++) {
        *floatPointer++ = *(float*)(floatLookUpTableBufferBase + ((*(short*)src)&0x0fff));
        src += 2;
    }
}

void DisparityConvertor::convertDisparity2RGB(const float* disparity, int width, int height, int minDisp, int maxDisp, unsigned char* rgbBuf)
{
    for (int y = 0; y<width; y++) {
        const float* pimg = disparity + y*height;
        unsigned char* pColor = rgbBuf + y*height*3;

        for (int x = 0; x<height; x++) {
            int baseByte = 3 * x;
            float temp = pimg[x];

            if (temp <= minDisp) {
                temp = 0.0f;
            } else if (temp >= maxDisp) {
                temp = 255.0f;
            } else {
                temp = float(temp - minDisp) / (float(maxDisp - minDisp)) * 255.0f;
            }

            if (temp<0.000001) {
                pColor[baseByte + 2] = 0;
                pColor[baseByte + 1] = 0;
                pColor[baseByte] = 0;
            } else if (temp <= 51) {
                pColor[baseByte + 2] = 255;
                pColor[baseByte + 1] = (unsigned char)(temp * 5 + 0.5);
                pColor[baseByte] = 0;
            } else if (temp <= 102) {
                pColor[baseByte + 2] = 255 - (unsigned char)((temp - 51) * 5 + 0.5);
                pColor[baseByte + 1] = 255;
                pColor[baseByte] = 0;
            } else if (temp <= 153) {
                pColor[baseByte + 2] = 0;
                pColor[baseByte + 1] = 255;
                pColor[baseByte] = (unsigned char)((temp - 102) * 5 + 0.5);
            } else if (temp <= 204) {
                pColor[baseByte + 2] = 0;
                pColor[baseByte + 1] = 255 - (unsigned char)(128.0*(temp - 153) / 51.0 + 0.5);
                pColor[baseByte] = 255;
            } else {
                pColor[baseByte + 2] = 0;
                pColor[baseByte + 1] = 127 - (unsigned char)(127.0*(temp - 204) / 51.0 + 0.5);
                pColor[baseByte] = 255;
            }
        }
    }
}

int DisparityConvertor::getDisparityBitNum(int format)
{
    int bitNum = 0;
    switch (format) {
    case FrameFormat::Disparity16:
    case FrameFormat::DisparityDens16:
        bitNum = 5;
        break;
    default:
        break;
    }
    return bitNum;
}
