#include <iostream>
#include <math.h>
#include <cstdlib>
#include <string>

#include "mycamerahandler.h"
#include "satpext.h"
#include "frameid.h"
#include "disparityconvertor.h"

#ifdef _WIN64
static const std::string k3dPointByPointFilePath = "D:/3d_pointbypoint.txt";
static const std::string k3dByLookUpTableFilePath = "D:/3d_by_lookuptable.txt";
static const std::string kRotationMartrixFilePath = "D:/rotation_matrix.txt";
#else
static const std::string kHomeDir = getenv("HOME") ? getenv("HOME") : "/var";
static const std::string k3dPointByPointFilePath = kHomeDir + "/3d_pointbypoint.txt";
static const std::string k3dByLookUpTableFilePath = kHomeDir + "/3d_by_lookuptable.txt";
static const std::string kRotationMartrixFilePath = kHomeDir + "/rotation_matrix.txt";
#endif

static const int kDisparityCount = 81;

MyCameraHandler::MyCameraHandler(std::string name):
    mName(name),
    mIsCalibParamReady(false)
{
    mIsLookupTableGenerated = false;
}

void MyCameraHandler::setStereoCalibParams(StereoCalibrationParameters &params)
{
    mStereoCalibrationParameters = params;
    mIsCalibParamReady = true;
    std::cout  << "calib params is ready!!!" << std::endl;
}

void MyCameraHandler::handleRawFrame(const RawImageFrame *rawFrame)
{
//    std::cout << mName
//              << ", got image, id: " << rawFrame->frameId
//              << " , time stamp: " << rawFrame->time
//              << std::endl;

    // put you image processing logic here.    eg:
    processFrame(rawFrame->frameId, (char*)rawFrame + sizeof(RawImageFrame),
                 rawFrame->dataSize, rawFrame->width, rawFrame->height, rawFrame->format);
}

void MyCameraHandler::processFrame(int frameId, char *image, uint32_t dataSize, int width, int height, int frameFormat)
{
    switch (frameId) {
    case FrameId::Disparity:
    {
        int bitNum = DisparityConvertor::getDisparityBitNum(frameFormat);
        if (mIsCalibParamReady) {
            // you can choose any of these methods
            handleDisparityByLookupTable((unsigned char *)image, width, height, bitNum);
//            handleDisparityPointByPoint((unsigned char *)image, width, height, bitNum);
        } else {
            std::cout  << "calib params is not ready!!!" << std::endl;
        }
    }
        break;
    default:
        break;
    }
}

void MyCameraHandler::handleDisparityPointByPoint(unsigned char *image, int width, int height, int bitNum)
{
    // get the disparity coordinate
    std::cout << "width: " << width << ", height: " << height << std::endl;

    // convert disparity format to float:
    static float *floatData = new float[width * height];
    DisparityConvertor::convertDisparity2FloatFormat(image, width, height, bitNum, floatData);

    // get the 3D point cloud data, and save one to file for verification
    static int index = 0;
    index ++;
    FILE * fp = nullptr;
    if (index == 1) {
        fp = fopen(k3dPointByPointFilePath.data(), "wb+");
        if (!fp) {
            std::cout << k3dPointByPointFilePath << " file not open" << std::endl;
            return;
        }
    }

    for(int posY = 0; posY < height; posY++) {
        for(int posX = 0; posX < width; posX++) {
            float x, y, z;
            DisparityConvertor::getPointXYZDistance(image, width, height, bitNum, mStereoCalibrationParameters.Tx,
                                                    mStereoCalibrationParameters.focus, mStereoCalibrationParameters.cx,
                                                    mStereoCalibrationParameters.cy, posX, posY, x, y, z);

            if((fabs(x) > 200000.0f)||(fabs(y) > 200000.0f)||(fabs(z) > 200000.0f)) {
                x = 0.0f;
                y = 0.0f;
                z = 0.0f;
            }
            if (index == 1) {
                fprintf(fp, "%f %f %f %d\n", x, y, z, (posY * width + posX));
            }
        }
    }
    if (index == 1) {
        fclose(fp);
    }
}

void MyCameraHandler::handleDisparityByLookupTable(unsigned char *image, int width, int height, int bitNum)
{
    std::cout << "width: " << width << ", height: " << height << std::endl;

    // generate X Y Z lookup table, only once is OK
    static float *lookUpTableX = new float[kDisparityCount*(int)pow(2, bitNum)*width];
    static float *lookUpTableY = new float[kDisparityCount*(int)pow(2, bitNum)*height];
    static float *lookUpTableZ = new float[kDisparityCount*(int)pow(2, bitNum)];
    if(!mIsLookupTableGenerated) {
        DisparityConvertor::generateLookUpTableX(width, bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.cx, lookUpTableX);
        DisparityConvertor::generateLookUpTableY(height, bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.cy, lookUpTableY);
        DisparityConvertor::generateLookUpTableZ(bitNum, mStereoCalibrationParameters.Tx, mStereoCalibrationParameters.focus, lookUpTableZ);
        mIsLookupTableGenerated = true;
    }

    // get X Y Z distance according to the lookup table
    float *xDistance = new float[width*height];
    DisparityConvertor::getWholeXDistanceByLookupTable(image, width, height, bitNum, lookUpTableX, xDistance);
    float *yDistance = new float[width*height];
    DisparityConvertor::getWholeYDistanceByLookupTable(image, width, height, bitNum, lookUpTableY, yDistance);
    float *zDistance = new float[width*height];
    DisparityConvertor::getWholeZDistanceByLookupTable(image, width, height, lookUpTableZ, zDistance);

    // get the 3D point cloud data, and save one to file for verification
    static int index = 0;
    index ++;
    FILE * fp = nullptr;
    if (index == 1) {
        fp = fopen(k3dByLookUpTableFilePath.data(), "wb+");
        if (!fp) {
            std::cout << k3dByLookUpTableFilePath << " file not open" << std::endl;
            return;
        }
    }
    for(int i = 0; i < width*height; i++) {
        float x, y, z;
        x = xDistance[i];
        y = yDistance[i];
        z = zDistance[i];
        if((fabs(x) > 200000.0f)||(fabs(y) > 200000.0f)||(fabs(z) > 200000.0f)) {
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }
        if (index == 1) {
            fprintf(fp, "%f %f %f %d\n", x, y, z, i);
        }
    }
    if (index == 1) {
        fclose(fp);
    }

    delete [] xDistance;
    delete [] yDistance;
    delete [] zDistance;
}
