#ifndef MYIMAGEHANDLER_H
#define MYIMAGEHANDLER_H

#include "camerahandler.h"
#include "calibrationparams.h"

#include <queue>
#include <mutex>
#include <condition_variable>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class MyCameraHandler: public CameraHandler
{
public:
    MyCameraHandler(const std::string &name);
    virtual ~MyCameraHandler();

    void handleRawFrame(const RawImageFrame *rawFrame);
    void setStereoCalibParams(StereoCalibrationParameters &params);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();

protected:
    void processFrame(int frameId, char *image, uint32_t dataSize, int width, int height, int frameFormat);
    void handleDisparityPointByPoint(unsigned char *image, int width, int height, int bitNum);
    void handleDisparityByLookupTable(unsigned char *image, int width, int height, int bitNum);

private:
    std::string name_;
    StereoCalibrationParameters calib_params_;

    bool is_lookup_table_generated_;
    bool is_calib_param_ready_;

    float *x_distance_buffer_;
    float *y_distance_buffer_;
    float *z_distance_buffer_;

    std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_queue_;
    std::mutex mutex_;
    std::condition_variable cloud_ready_cond_;
    std::condition_variable cloud_queue_spare_cond_;
};

#endif // MYIMAGEHANDLER_H
