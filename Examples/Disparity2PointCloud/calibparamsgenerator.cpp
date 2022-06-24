#include "calibparamsgenerator.h"
#include "opencv2/opencv.hpp"

CalibParamsGenerator::CalibParamsGenerator()
{

}

bool CalibParamsGenerator::generateStereoParameters(const cv::Mat *calibData, const cv::Mat *depthData, const cv::Mat *autoCalib, StereoCalibrationParameters &params, std::string calibFileVer)
{
    std::cout << "calibData file version is: " << calibFileVer << std::endl;
    if (calibFileVer == "0.0") {
        return generateStereoParametersVer0_0(calibData, depthData, autoCalib, params);
    } else if (calibFileVer == "1.0") {
        return generateStereoParametersVer1_0(calibData, depthData, autoCalib, params);
    } else {
        std::cout << "The file version is not recognized!" << std::endl;
        return false;
    }
}

bool CalibParamsGenerator::generateStereoParametersVer0_0(const cv::Mat *calibData, const cv::Mat *depthData, const cv::Mat *autoCalib, StereoCalibrationParameters &params)
{
    if ((nullptr != calibData) && (!calibData->empty())) {
        params.focus = ((calibData->at<double>(0,4) < calibData->at<double>(0,12))? calibData->at<double>(0,4) : calibData->at<double>(0,12));
        params.cx = (calibData->at<double>(0,39) + calibData->at<double>(0,41))/2.0;
        params.cy = (calibData->at<double>(0,40) + calibData->at<double>(0,42))/2.0;
        params.RRoll = 0;
        params.RPitch = 0;
        params.RYaw = 0;
        params.Tx = fabs(calibData->at<double>(0,46));
        params.Ty = 0;
        params.Tz = 0;
    }
    else
    {
        return false;
    }


    if ((nullptr != depthData) && (!depthData->empty())) {
        double deltaD = 0.0;
        deltaD = depthData->at<double>(0,4) + depthData->at<double>(0,0);
        params.Tx = fabs(depthData->at<double>(0,2)*1000.0);
        params.cx = (calibData->at<double>(0,39) + calibData->at<double>(0,41))/2.0 - floor(deltaD);
    }
    else
    {
        return false;
    }


    if ((nullptr != autoCalib) && (!autoCalib->empty())) {
        params.cx = (calibData->at<double>(0,39) + calibData->at<double>(0,41))/2.0 - floor(autoCalib->at<double>(0,8));
        params.Tx = fabs(autoCalib->at<double>(0,3));
    }

    return true;
}

bool CalibParamsGenerator::generateStereoParametersVer1_0(const cv::Mat *calibData, const cv::Mat *depthData, const cv::Mat *autoCalib, StereoCalibrationParameters &params)
{
    if ((nullptr != calibData) && (!calibData->empty())) {
        params.focus = calibData->at<double>(0,41);
        params.cx = calibData->at<double>(0,42);
        params.cy = calibData->at<double>(0,43);
        params.RRoll = 0;
        params.RPitch = 0;
        params.RYaw = 0;
        params.Tx = fabs(calibData->at<double>(0,22));
        params.Ty = 0;
        params.Tz = 0;
    }
    else
    {
        return false;
    }

    if ((nullptr != depthData) && (!depthData->empty())) {
        double deltaD = 0.0;
        deltaD = depthData->at<double>(0,4) + depthData->at<double>(0,0);
        params.Tx = fabs(depthData->at<double>(0,2)*1000.0);
        params.cx = calibData->at<double>(0,42) - floor(deltaD);
    }
    else
    {
        return false;
    }

    if ((nullptr != autoCalib) && (!autoCalib->empty())) {
        params.cx = calibData->at<double>(0,42) - floor(autoCalib->at<double>(0,8));
        params.Tx = fabs(autoCalib->at<double>(0,3));
    }

    return true;
}

void CalibParamsGenerator::generateMonoParameters(const cv::Mat &M1Data, const cv::Mat &D1Data, MonoCalibrationParameters &params)
{
    params.fx = M1Data.at<double>(0,0);
    params.fy = M1Data.at<double>(1,1);
    params.cx = M1Data.at<double>(0,2);
    params.cy = M1Data.at<double>(1,2);
    params.k1 = D1Data.at<double>(0,0);
    params.k2 = D1Data.at<double>(0,1);
    params.k3 = 0;
    params.p1 = D1Data.at<double>(0,2);
    params.p2 = D1Data.at<double>(0,3);
}
