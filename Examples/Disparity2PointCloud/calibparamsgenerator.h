#ifndef CALIBPARAMSGENERATOR_H
#define CALIBPARAMSGENERATOR_H

#include <string>
#include "calibrationparams.h"


namespace cv
{
    class Mat;
}

class CalibParamsGenerator
{

public:
    CalibParamsGenerator();
    static bool generateStereoParameters(const cv::Mat *calibData, const cv::Mat *depthData, const cv::Mat *autoCalib,
                                         StereoCalibrationParameters &params, std::string calibFileVer);
    static bool generateStereoParametersVer0_0(const cv::Mat *calibData, const cv::Mat *depthData, const cv::Mat *autoCalib,
                                                StereoCalibrationParameters &params);
    static bool generateStereoParametersVer1_0(const cv::Mat *calibData, const cv::Mat *depthData, const cv::Mat *autoCalib,
                                                StereoCalibrationParameters &params);
    static void generateMonoParameters(const cv::Mat &M1Data, const cv::Mat &D1Data, MonoCalibrationParameters &params);
};

#endif // CALIBPARAMSGENERATOR_H
