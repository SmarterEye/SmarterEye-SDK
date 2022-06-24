#ifndef CONVERTORMANAGER_H
#define CONVERTORMANAGE_H

#include <QThread>
#include <QRunnable>
#include <QMutex>
#include <QDirIterator>
#include <opencv2/opencv.hpp>

class StereoCalibrationParameters;
class DisparityFrameConvertor;

class ConvertorManage : public QThread
{
public:
    ConvertorManage(const QString filepath, DisparityFrameConvertor * dispatityFrameVentor);
    ~ConvertorManage() override;
    bool requestStereoCameraParams();
    static bool readMat(QString key, QString fileName, cv::Mat &pdata);
    static void readFileVersion(QString fileName, std::string &version);

    bool getStereoCameraParams(StereoCalibrationParameters &calibParams, QString primaryPath = "", QString advancedPath = "", QString autoPath = "");

    const QString &getInputDir() const {return mInputDir;}
    const QString &getOutputDir() const {return mOutputDir;}

    QDirIterator *getDirIterator() {return mDirIterator;}
    QMutex *getThreadMutex(){return &mMutex;}
    QStringList getDirFilters(){return mDirFilters;}

    DisparityFrameConvertor *mDisparityFrameConvertor;


protected:
    void run() override {startConvert();}
    void startConvert();

private:
    const QString kPrimaryMarkedCameraParamsFilePath;
    const QString kAdvancedMarkedCameraParamsFilePath;
    const QString kOutdoorMarkedCameraParamsFilePath;
    const QString kAutoMarkedCameraParamsFilePath;
    QString mInputDir;
    QString mOutputDir;
    QDirIterator *mDirIterator;
    QStringList mDirFilters;
    QMutex mMutex;  

};

class ConvertThread : public QRunnable
{
public:
    explicit ConvertThread(ConvertorManage *convertorManager);
    ~ConvertThread() override ;
protected:
    bool dirIteratorNotEnd();
    void run() override;

private:
    ConvertorManage *m_pConvertorManage;
};

#endif // CONVERTORMANAGE_H
