#include <ros/ros.h>
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Imu.h"

#include "zkhy_stereo_d/CameraParams.h"
#include "zkhy_stereo_d/RotationMatrix.h"

#include <opencv2/opencv.hpp>

class StereoListener
{
public:
    StereoListener();
    ~StereoListener();

    void getCameraParams();
    void getRotationMatrix();

    void grayCallback(const sensor_msgs::ImageConstPtr &msg);
    void colorCallback(const sensor_msgs::ImageConstPtr &msg);
    void disparityCallback(const sensor_msgs::ImageConstPtr &msg);
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);

private:
    ros::NodeHandle node_handler_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber gray_sub_;
    image_transport::Subscriber color_sub_;
    image_transport::Subscriber disparity_sub_;
    ros::Subscriber imu_sub_;

    ros::ServiceClient get_camera_params_client_;
    ros::ServiceClient get_rotation_matrix_client_;
};

StereoListener::StereoListener()
        : it_(node_handler_)
{
    gray_sub_ = it_.subscribe("/zkhy_stereo/left/gray", 1, &StereoListener::grayCallback, this);
    color_sub_ = it_.subscribe("/zkhy_stereo/left/color", 1, &StereoListener::colorCallback, this);
    disparity_sub_ = it_.subscribe("/zkhy_stereo/disparity", 1, &StereoListener::disparityCallback, this);
    imu_sub_ = node_handler_.subscribe("/zkhy_stereo/imu", 1, &StereoListener::imuCallback, this);

    get_camera_params_client_ = node_handler_.serviceClient<zkhy_stereo_d::CameraParams>("/zkhy_stereo/get_camera_params");
    get_rotation_matrix_client_ = node_handler_.serviceClient<zkhy_stereo_d::RotationMatrix>("/zkhy_stereo/get_rotation_matrix");

    cv::namedWindow("gray");
    cv::namedWindow("color");
    cv::namedWindow("disparity");
}

StereoListener::~StereoListener()
{
    cv::destroyAllWindows();
}

void StereoListener::grayCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr image_ptr;
    try {
        image_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (cv_bridge::Exception &e) {
        return;
    }

    cv::imshow("gray", image_ptr->image);
    cv::waitKey(80);
}

void StereoListener::colorCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr image_ptr;
    try {
        image_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception &e) {
        return;
    }

    cv::Mat img = image_ptr->image;
    cv::cvtColor(img, img, CV_RGB2BGR);
    cv::imshow("color", img);
    cv::waitKey(80);
}

void StereoListener::disparityCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr image_ptr;
    try {
        image_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
        return;
    }

    cv::imshow("disparity", image_ptr->image);
    cv::waitKey(80);
}

void StereoListener::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    ROS_INFO("Imu stamp: [%d]", msg->header.stamp);
    ROS_INFO("Imu linear_acceleration x: [%f], y: [%f], z: [%f]",
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);
    ROS_INFO("Imu angular_velocity x: [%f], y: [%f], z: [%f]",
             msg->angular_velocity.x,
             msg->angular_velocity.y,
             msg->angular_velocity.z);
}

void StereoListener::getCameraParams()
{
    // no request param
    zkhy_stereo_d::CameraParams svr;
    bool ok = get_camera_params_client_.call(svr);
    if (ok) {
        auto camera_params_resp = svr.response;
        // got camera params here
        std::cout << "camera focus: " << camera_params_resp.focus << std::endl;
    } else {
        std::cout << "error getCameraParams" << std::endl;
    }
}

void StereoListener::getRotationMatrix()
{
    zkhy_stereo_d::RotationMatrix svr;
    bool ok = get_rotation_matrix_client_.call(svr);
    if (ok) {
        auto rotation_matrix_resp = svr.response;
        // got camera params here
        std::cout << "rotation matrix: " << rotation_matrix_resp.real3DToImage[0] << std::endl;
    } else {
        std::cout << "error bar" << std::endl;
    }
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "stereo_listener_d");

    StereoListener stereo_listener;
    stereo_listener.getCameraParams();
    stereo_listener.getRotationMatrix();

    ros::spin();

    return 0;
}
