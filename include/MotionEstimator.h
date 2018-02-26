//
// Created by roman on 26.02.18.
//

#ifndef MONOCULAR_ODOMETRY_MOTIONESTIMATOR_H
#define MONOCULAR_ODOMETRY_MOTIONESTIMATOR_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class MotionEstimator
{
public:
    MotionEstimator(const char* parameters);
    ~MotionEstimator() {}
    void processFrame(const sensor_msgs::ImageConstPtr& msg);
    void findCameraPose(cv_bridge::CvImagePtr & image, float time_threshold);
    cv::Mat least_squares_method(const cv::Mat & diff, const cv::Mat & grad_x, const cv::Mat & grad_y);
    void updateRotationMatrix(Mat & rotation_matrix, const float pitch, const float roll, const float yaw);
private:
    double focus_;
    double pixel_size_;
    double height_;

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subsciber sub_;

    cv::Mat previous_frame_;

    geometry_msgs::Pose camera_pose_;
};

#endif //MONOCULAR_ODOMETRY_MOTIONESTIMATOR_H
