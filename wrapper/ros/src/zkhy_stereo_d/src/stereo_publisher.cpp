//
// Created by xtp on 2020/3/9.
//

#include "stereo_publisher.h"

#include "framemonitor.h"
#include "stereocamera.h"
#include "frameid.h"
#include "calibrationparams.h"
#include "rotationmatrix.h"

#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include <cmath>

static const std::string kLeftGrayTopic("/zkhy_stereo/left/gray");
static const std::string kLeftColorTopic("/zkhy_stereo/left/color");

static const std::string kRigthGrayTopic("/zkhy_stereo/right/gray");
static const std::string kRigthColorTopic("/zkhy_stereo/right/color");

static const std::string kDisparityTopic("/zkhy_stereo/disparity");

static const std::string kImuTopic("/zkhy_stereo/imu");
static const std::string kPointCloudTopic("/zkhy_stereo/points");

static const std::string kCameraParamsService("/zkhy_stereo/get_camera_params");
static const std::string kRotationMatrixService("/zkhy_stereo/get_rotation_matrix");
static const std::string kFrameRateService("/zkhy_stereo/set_frame_rate");

StereoPublisher::StereoPublisher()
    : it_(node_handler_),
      stereo_camera_(StereoCamera::connect("192.168.1.251")),
      frame_monitor_(new FrameMonitor)
{
    bool isConnected = false;
	// frame rate
    float rate = 0.0;
    while (!isConnected) {
        printf("connecting...\n");
        isConnected = stereo_camera_->isConnected();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if(stereo_camera_->getFrameRate(rate))
    {
        std::cout <<"frame rate : "<<rate<<std::endl;
    }
    else
    {
        std::cout << "get frame rate failed" <<std::endl;
    }
    left_gray_pub_ = it_.advertise(kLeftGrayTopic, 1);
    left_color_pub_ = it_.advertise(kLeftColorTopic, 1);

    right_gray_pub_ = it_.advertise(kRigthGrayTopic, 1);
    right_color_pub_ = it_.advertise(kRigthColorTopic, 1);

    disparity_pub_ = it_.advertise(kDisparityTopic, 1);

    imu_pub_ = node_handler_.advertise<sensor_msgs::Imu>(kImuTopic, 100);
    points_pub_ = node_handler_.advertise<sensor_msgs::PointCloud2>(kPointCloudTopic, 1);

    camera_params_server_ = node_handler_.advertiseService(kCameraParamsService, &StereoPublisher::onCameraParamsRequest, this);
    rotation_matrix_server_ = node_handler_.advertiseService(kRotationMatrixService, &StereoPublisher::onRotationMatrixRequest, this);
    frame_rate_server_ = node_handler_.advertiseService(kFrameRateService, &StereoPublisher::onFrameRateSetRequest, this);

    stereo_camera_->requestFrame(frame_monitor_.get(), FrameId::CalibLeftCamera | FrameId::CalibRightCamera | FrameId::Disparity);
    stereo_camera_->enableMotionData(true);

    StereoCalibrationParameters params {};
    stereo_camera_->requestStereoCameraParameters(params);

    frame_monitor_->setStereoCalibParams(params);
    frame_monitor_->setFrameCallback(std::bind(&StereoPublisher::publishFrameCallback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    frame_monitor_->setMotionDataCallback(std::bind(&StereoPublisher::publishImuCallback, this, std::placeholders::_1));
    frame_monitor_->setPointCloudCallback(std::bind(&StereoPublisher::publishPointsCallback, this, std::placeholders::_1));
}

StereoPublisher::~StereoPublisher()
{
    stereo_camera_->disconnectFromServer();
}

bool StereoPublisher::onCameraParamsRequest(zkhy_stereo_d::CameraParams::Request &req,
                                            zkhy_stereo_d::CameraParams::Response &resp)
{
    StereoCalibrationParameters params {};
    bool ok = stereo_camera_->requestStereoCameraParameters(params);
    if (ok) {
        resp.focus = params.focus;
        resp.cx = params.cx;
        resp.cy = params.cy;
        resp.RRoll = params.RRoll;
        resp.RPitch = params.RPitch;
        resp.RYaw = params.RYaw;
        resp.Tx = params.Tx;
        resp.Ty = params.Ty;
        resp.Tz = params.Tz;
    } else {
        std::cout << "Stereo camera params request failed from ADAS.";
    }

    return ok;
}

bool StereoPublisher::onRotationMatrixRequest(zkhy_stereo_d::RotationMatrix::Request &req,
                                              zkhy_stereo_d::RotationMatrix::Response &resp)
{
    RotationMatrix matrix {};
    bool ok = stereo_camera_->requestRotationMatrix(matrix);
    if (ok) {
        for (size_t i = 0; i < resp.real3DToImage.size(); i++) {
            resp.real3DToImage[i] = matrix.real3DToImage[i];
        }
        for (size_t j = 0; j < resp.imageToReal3D.size(); j++) {
            resp.imageToReal3D[j] = matrix.imageToReal3D[j];
        }
    } else {
        std::cout << "Rotation matrix request failed from ADAS.";
    }

    return ok;
}

bool StereoPublisher::onFrameRateSetRequest(zkhy_stereo_d::FrameRate::Request &req,zkhy_stereo_d::FrameRate::Response &resp)
{
    float rate = req.rate;
    std::cout << " request rate :"<<rate<<std::endl;
    bool ok = stereo_camera_->setFrameRate(rate);
    if(ok)
    {
        std::cout << " set frame rate done"<<std::endl;
    }
    else
    {
        std::cout << " set frame rate failed"<<std::endl;
    }
    return ok;
}

void StereoPublisher::publishFrameCallback(int frameId, int64_t timestamp, const cv::Mat &img_mat)
{
    bool is_color = (img_mat.type() == CV_8UC3);
    std::string encoding = is_color ? "rgb8" : "mono8";
    sensor_msgs::ImagePtr img_msg;

    if (frameId == FrameId::CalibLeftCamera || frameId == FrameId::CalibRightCamera) {
        img_msg = cv_bridge::CvImage(std_msgs::Header(), encoding, img_mat).toImageMsg();
        img_msg->header.stamp = ros::Time(timestamp / 1000.0);

        if (frameId == FrameId::CalibLeftCamera) {
            if (is_color) {
                img_msg->header.frame_id = "zkhy_left_color_frame";
                left_color_pub_.publish(img_msg);
            } else {
                img_msg->header.frame_id = "zkhy_left_gray_frame";
                left_gray_pub_.publish(img_msg);
            }
        }

        if (frameId == FrameId::CalibRightCamera) {
            if (is_color) {
                img_msg->header.frame_id = "zkhy_right_color_frame";
                right_color_pub_.publish(img_msg);
            } else {
                img_msg->header.frame_id = "zkhy_right_gray_frame";
                right_gray_pub_.publish(img_msg);
            }
        }
    }

    if (frameId == FrameId::Disparity) {
        img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_mat).toImageMsg();
        img_msg->header.frame_id = "zkhy_disparity_frame";
        disparity_pub_.publish(img_msg);
    }
}

void StereoPublisher::publishImuCallback(const MotionData *motionData)
{
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time(motionData->timestamp / 1000.0);
    imu_msg.header.frame_id = "zkhy_imu_frame";

    imu_msg.linear_acceleration.x = motionData->accelX;
    imu_msg.linear_acceleration.y = motionData->accelY;
    imu_msg.linear_acceleration.z = motionData->accelZ;

    imu_msg.angular_velocity.x = motionData->gyroX * M_PI / 180;
    imu_msg.angular_velocity.y = motionData->gyroY * M_PI / 180;
    imu_msg.angular_velocity.z = motionData->gyroZ * M_PI / 180;

    imu_pub_.publish(imu_msg);
}

void StereoPublisher::publishPointsCallback(sensor_msgs::PointCloud2::Ptr cloud)
{
    cloud->header.frame_id = "zkhy_points";
    points_pub_.publish(cloud);
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "stereo_publisher_d");

    StereoPublisher stereo_publisher;

    ros::spin();

    return 0;
}
