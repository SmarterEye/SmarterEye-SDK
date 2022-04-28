//
// Created by xtp on 2020/3/9.
//

#ifndef ZKHY_STEREO_D_STEREO_PUBLISHER_H
#define ZKHY_STEREO_D_STEREO_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

#include "zkhy_stereo_d/CameraParams.h"
#include "zkhy_stereo_d/RotationMatrix.h"
#include "zkhy_stereo_d/FrameRate.h"

class StereoCamera;
class FrameMonitor;
struct MotionData;
namespace cv{
    class Mat;
}

class StereoPublisher
{
public:
    StereoPublisher();
    ~StereoPublisher();

    bool onCameraParamsRequest(zkhy_stereo_d::CameraParams::Request &req,
                               zkhy_stereo_d::CameraParams::Response &resp);

    bool onRotationMatrixRequest(zkhy_stereo_d::RotationMatrix::Request &req,
                               zkhy_stereo_d::RotationMatrix::Response &resp);
  
    bool onFrameRateSetRequest(zkhy_stereo_d::FrameRate::Request &req,
                              zkhy_stereo_d::FrameRate::Response &resp);

    void publishFrameCallback(int frameId, int64_t timestamp, const cv::Mat &img_mat);

    void publishImuCallback(const MotionData *motionData);

    void publishPointsCallback(sensor_msgs::PointCloud2::Ptr cloud);

private:
    ros::NodeHandle node_handler_;
    image_transport::ImageTransport it_;
    image_transport::Publisher left_gray_pub_;
    image_transport::Publisher left_color_pub_;
    image_transport::Publisher right_gray_pub_;
    image_transport::Publisher right_color_pub_;
    image_transport::Publisher disparity_pub_;

    ros::Publisher imu_pub_;
    ros::Publisher points_pub_;

    ros::ServiceServer camera_params_server_;
    ros::ServiceServer rotation_matrix_server_;
    ros::ServiceServer frame_rate_server_;

    std::unique_ptr<StereoCamera> stereo_camera_;
    std::unique_ptr<FrameMonitor> frame_monitor_;
};


#endif //ZKHY_STEREO_D_STEREO_PUBLISHER_H
