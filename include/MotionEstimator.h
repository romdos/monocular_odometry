#ifndef MONOCULAR_ODOMETRY_MOTIONESTIMATOR_H
#define MONOCULAR_ODOMETRY_MOTIONESTIMATOR_H
// Standard libraries
#include <fstream>
#include <boost/algorithm/string.hpp>
// ROS
#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

class MotionEstimator
{
public:
    MotionEstimator(const char* parameters);
    ~MotionEstimator() {} // todo: is it needable here?

    void processFrame(const sensor_msgs::ImageConstPtr& msg);
    void findCameraPose(cv_bridge::CvImagePtr & image, float time_threshold);
    cv::Mat least_squares_method(const cv::Mat & diff);
    void updateRotationMatrix(const float pitch, const float roll, const float yaw);

private:
    float focus_;
    float pixel_size_;
    float height_;

//    image_transport::Publisher pub_;
    double focus_;
    double pixel_size_;
    double height_;

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    cv::Mat previous_frame_;
    cv::Mat grad_x_, grad_y_;
    cv::Mat_<float> rotation_matrix_;

    double time_now_;

    geometry_msgs::PoseStamped camera_pose_;
};

#endif //MONOCULAR_ODOMETRY_MOTIONESTIMATOR_H
