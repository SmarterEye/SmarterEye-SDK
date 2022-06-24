#include<iostream>
#include "convertormanage.h"
#include <QThreadPool>
#include "disparityframeconvertor.h"
#include <QDebug>
#include <QString>
#include "calibparamsgenerator.h"
#include "common.h"

ConvertorManage::ConvertorManage(const QString filepath, DisparityFrameConvertor *dispatityFrameVentor):
    mDisparityFrameConvertor(dispatityFrameVentor),
    kPrimaryMarkedCameraParamsFilePath(filepath+"/adas_params/calibData.yml"),
    kAdvancedMarkedCameraParamsFilePath(filepath+"/adas_params/depthData.yml"),
    kOutdoorMarkedCameraParamsFilePath(filepath+"/adas_params/depthRefine.yml"),
    kAutoMarkedCameraParamsFilePath(filepath+"/adas_params/autoCalib.yml")

{
    mInputDir = filepath;
    mOutputDir = filepath;
    mDirFilters<<QString("disparity*.dat")<<QString("disparity*.png");
    connect(this, &ConvertorManage::finished, [this]{
            if (mDirIterator) {
                delete mDirIterator;
                mDirIterator = nullptr;
            }
        });
    connect(this,&ConvertorManage::finished,this,[=](){
            this->deleteLater();
        });
}

ConvertorManage::~ConvertorManage()
{
    delete mDisparityFrameConvertor;
}

bool ConvertorManage::requestStereoCameraParams()
{
    StereoCalibrationParameters calibParams;
    if(getStereoCameraParams(calibParams))
    {
        std::cout <<"CalibParam focus: "<<calibParams.focus<<std::endl;
        std::cout <<"CalibParam cx: "<<calibParams.cx<<std::endl;
        std::cout <<"CalibParam cy: "<<calibParams.cy<<std::endl;
        std::cout <<"CalibParam Tx: "<<calibParams.Tx<<std::endl;
        std::cout <<"CalibParam Ty: "<<calibParams.Ty<<std::endl;
        std::cout <<"CalibParam Tz: "<<calibParams.Tz<<std::endl;

        mDisparityFrameConvertor->setStereoCalibParams(calibParams);
        return true;
    }
    return false;
}

bool ConvertorManage::readMat(QString key, QString fileName, cv::Mat &pdata)
{
    if (QFile::exists(fileName)) {
            try{
                cv::FileStorage fs(fileName.toStdString(), cv::FileStorage::READ);
                fs[key.toStdString()] >> pdata;
                fs.release();
                return true;
            }catch (cv::Exception e){
                qWarning() << "got exception [" << e.err.c_str() << "] when reading"
                           << fileName;
            }
        }
        return false;
}

void ConvertorManage::readFileVersion(QString fileName, std::string &version)
{
    if (!QFile::exists(fileName)) return;

        cv::FileStorage fs(fileName.toStdString().c_str(), cv::FileStorage::READ);
        version = (std::string)fs["FileVersion"];
        if (version.empty()) version = "0.0";
        return;
}

bool ConvertorManage::getStereoCameraParams(StereoCalibrationParameters &calibParams, QString primaryPath, QString advancedPath, QString autoPath)
{
    cv::Mat calibrationData;
    cv::Mat depthCalibrationData;
    cv::Mat autoCalibrationData;
    cv::Mat *pCalibrationData = nullptr;
    cv::Mat *pDepthCalibrationData = nullptr;
    cv::Mat *pAutoCalibrationData = nullptr;
    std::string calibFileVer;

    if (primaryPath.isEmpty()) {
        primaryPath = kPrimaryMarkedCameraParamsFilePath;
    }
    if(!ConvertorManage::readMat("Calib", primaryPath, calibrationData)) {
        qDebug() << "no Calib Data found !!!";
        return false;
    } else {
        ConvertorManage::readFileVersion(primaryPath, calibFileVer);
    }
    pCalibrationData = &calibrationData;

    if (advancedPath.isEmpty()) {
        advancedPath = kOutdoorMarkedCameraParamsFilePath;
        if(!QFile::exists(advancedPath)) {
            advancedPath = kAdvancedMarkedCameraParamsFilePath;
        }
    }
    if(!ConvertorManage::readMat("DepthCalib", advancedPath, depthCalibrationData)) {
        qDebug() << "no DepthCalib Data found !!!";
        return false;
    }
    pDepthCalibrationData = &depthCalibrationData;

    if (autoPath.isEmpty()) {
        autoPath = kAutoMarkedCameraParamsFilePath;
    }
    if(ConvertorManage::readMat("AutoCalib", autoPath, autoCalibrationData)) {
        pAutoCalibrationData = &autoCalibrationData;
    } else {
        qDebug() << "no AutoCalib Data found !!!";
    }

    return CalibParamsGenerator::generateStereoParameters(pCalibrationData, pDepthCalibrationData, pAutoCalibrationData,
                                                   calibParams, calibFileVer);
}

void ConvertorManage::startConvert()
{
    Q_ASSERT(mInputDir.size() > 0);
    Q_ASSERT(mOutputDir.size() > 0);
    Q_ASSERT(mDirFilters.size() > 0);

    mDirIterator = new QDirIterator(mInputDir, mDirFilters, QDir::Files | QDir::NoSymLinks, QDirIterator::Subdirectories);
    QThreadPool::globalInstance()->start(new ConvertThread(this));
    QThreadPool::globalInstance()->waitForDone();
    qDebug()<<"Convert done!";
}

ConvertThread::ConvertThread(ConvertorManage *convertorManage):
    m_pConvertorManage(convertorManage)
{
    Q_ASSERT(m_pConvertorManage);
}

ConvertThread::~ConvertThread()
{
}

bool ConvertThread::dirIteratorNotEnd()
{
    QMutexLocker locker(m_pConvertorManage->getThreadMutex());
    return m_pConvertorManage->getDirIterator()->hasNext();
}

void ConvertThread::run()
{
    const QString inputDir = m_pConvertorManage->getInputDir();
    const QString outputDir = inputDir;
    while (dirIteratorNotEnd())
    {
        QDirIterator *dirIterator = nullptr;
        {
            QMutexLocker locker(m_pConvertorManage->getThreadMutex());
            dirIterator = m_pConvertorManage->getDirIterator();
            if (!dirIterator) break;

            dirIterator->next();
        }
        QFileInfo fileInfo = dirIterator->fileInfo();
        QString absolutePath = fileInfo.absolutePath();
        QString relPath = absolutePath.mid(inputDir.length());
        QString saveDirName;
        if(!outputDir.contains("output"))
        {
            saveDirName = outputDir + "/output";
        }
        saveDirName += relPath;
        if(!QDir(saveDirName).exists())
        {
            QDir().mkpath(saveDirName);
        }

        QString fileDest;
        int format = -1;
        if(fileInfo.suffix() == "dat" || fileInfo.suffix() == "png"){
            fileDest = saveDirName +"/"+fileInfo.fileName();
            if( fileInfo.suffix() == "dat")
            {
                format = FileFormat::dat;
            }
            else if(fileInfo.suffix() == "png")
            {
                format = FileFormat::png;
            }
        }
        if(m_pConvertorManage->mDisparityFrameConvertor)
        {
            fileDest = saveDirName + "/" + fileInfo.completeBaseName() + ".pcd";
            qDebug() << "generate :  " << fileDest;
            m_pConvertorManage->mDisparityFrameConvertor->convertOneFrame(fileInfo.absoluteFilePath(), fileDest, format);
        }

    }
}
