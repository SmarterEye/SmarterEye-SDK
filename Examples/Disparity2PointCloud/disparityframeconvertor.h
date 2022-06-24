#ifndef DISPARITYFRAMECONVERTOR_H
#define DISPARITYFRAMECONVERTOR_H

#include <QtCore/qobject.h>
#include <QString>
#include <condition_variable>
#include <queue>
#include <mutex>
#include "calibrationparams.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class  DisparityFrameConvertor
{   
  public:
    DisparityFrameConvertor();
    ~DisparityFrameConvertor();
    void convertOneFrame(const QString &fileSrc, const QString &fileDest, int format);
    int getBitNumByFileSize(qint64 fileSize) const;
    void setStereoCalibParams(StereoCalibrationParameters &params);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
    QString GetCloudPath();
    int GetQueueSize();
protected:
    void handleDisparityByLookupTable(unsigned char *image, int width, int height, int bitNum);

private:
    bool is_lookup_table_generated_;
    StereoCalibrationParameters calib_params_;
    float *x_distance_buffer_;
    float *y_distance_buffer_;
    float *z_distance_buffer_;
    std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_queue_;
    std::queue<QString> cloud_path_queue_;
    std::condition_variable cloud_ready_cond_;
    std::condition_variable cloud_queue_spare_cond_;
    std::mutex mutex_;

};

#endif // DISPARITYFRAMECONVERTOR_H
