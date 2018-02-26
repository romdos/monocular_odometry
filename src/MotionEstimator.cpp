//
// Created by roman on 26.02.18.
//

#include "MotionEstimator.h"

MotionEstimator::MotionEstimator(const char *parameters)
{
    // todo: read parameters: focus and pixel size

    pub_ = nh_.advertise<geometry_msgs::Pose>("transform", 1000);

    sub_ = nh_.subscribe("/camera/image_raw", 1, processFrame);

    pub_.publish(camera_pose_);
}

void MotionEstimator::processFrame(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    findCameraPose(cv_ptr, 0.0001);
}

void MotionEstimator::findCameraPose(cv_bridge::CvImagePtr& cv_ptr, float time_threshold)
{
    cv::Mat frame = cv_ptr -> image;
    /* Gradient estimation via Sobel operator */
    int scale = 1;
    int delta = 0;
    int ddepth = CV_32FC1;
    cv::Mat norm_frame;
    cv::Mat grad_x, grad_y;

    if (cv_ptr -> header.stamp.sec < time_threshold) // todo: adjust
    {
        previous_frame_ = cv_ptr -> image;
        previous_frame_.convertTo(previous_frame_, CV_32FC1, 1.0 / 255.0);

        Sobel(previous_frame_, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
        Sobel(previous_frame_, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);

        return;
    }

    cv::Vec3d translation(0, 0, 0);          // translation relative to initial coordinate frame
    cv::Vec3d current_translation(0, 0, 0); // translation between current consequent frames

    cv::Mat rotation_matrix         = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat current_rotation_matrix = cv::Mat::eye(3, 3, CV_32FC1);

    frame.convertTo(norm_frame, CV_32FC1, 1.0 / 255.0);

    cv::Mat diff = norm_frame - previous_frame_;
    /* Least Squares Method for transform parameters finding */
    cv::Mat transform_params = least_squares_method(diff, grad_x, grad_y);
    /* Translation vector */
    current_translation = cv::Vec3f(transform_params.at<float>(0, 0),
                                transform_params.at<float>(1, 0),
                                transform_params.at<float>(2, 0));
    /* Rotation */
    updateRotationMatrix(rotation_matrix, transform_params.at<float>(3, 0),
                         transform_params.at<float>(4, 0),
                         transform_params.at<float>(5, 0));

    translation = translation + rotation_matrix * current_translation;
    // todo: make a camera_pose!
    previous_frame_ = norm_frame.clone();

    Sobel(previous_frame_, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    Sobel(previous_frame_, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);

}

cv::Mat MotionEstimator::least_squares_method(const cv::Mat &diff, const cv::Mat &grad_x, const cv::Mat &grad_y)
{
    int image_width  = diff.cols;
    int image_height = diff.rows;

    float k = 0.1; // todo: pixel size: mm / px

    // todo: sum over ROI !
    float alpha = 1.0;
    auto vertical_level = (int) (alpha * image_height / 2 + 1);
    int points_number = vertical_level * image_width;

    cv::Mat A = cv::Mat::zeros(points_number, 6, CV_32FC1);
    cv::Mat b = cv::Mat::zeros(points_number, 1, CV_32FC1);

    for (int i = 0; i < vertical_level; i++)
    {
        for (int j = 0; j < image_width; j++)
        {
            float dI = diff.at<float>(i, j);
            if (dI < 0)
                dI = 0;

            float I_x = grad_x.at<float>(i, j);
            float I_y = grad_y.at<float>(i, j);

            float y_horizont = k * vertical_level;

            float y = y_horizont + k * (i + 1);
            float x = k * j;

            float Z = focus_ * height_ / (y - y_horizont);

            A.at<float>(i * image_width + j, 0) = -focus_ * I_x;
            A.at<float>(i * image_width + j, 1) = -focus_ * I_y;
            A.at<float>(i * image_width + j, 2) = (I_x * x + I_y * y) / Z;
            A.at<float>(i * image_width + j, 3) = I_x * x * y / focus_ + I_y * (focus_ + y * y / focus_);
            A.at<float>(i * image_width + j, 4) = -(focus_ + x * x / focus_) * I_x - I_y * x * y / focus_;
            A.at<float>(i * image_width + j, 5) = I_x * y - I_y * x;

            b.at<float>(i * image_width + j, 0) = dI;
        }
    }

    return (A.t() * A).inv() * A.t() * b;

}

void updateRotationMatrix(cv::Mat & rotation_matrix, const float pitch, const float roll, const float yaw)
{
    /* Transform matrix construction */
    cv::Mat pitch_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat roll_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat yaw_matrix = cv::Mat::zeros(3, 3, CV_32FC1);

    pitch_matrix.at<float>(0, 0) = 1;
    pitch_matrix.at<float>(0, 1) = 0;
    pitch_matrix.at<float>(0, 2) = 0;
    pitch_matrix.at<float>(1, 0) = 0;
    pitch_matrix.at<float>(2, 0) = 0;

    roll_matrix.at<float>(0, 1) = 0;
    roll_matrix.at<float>(1, 0) = 0;
    roll_matrix.at<float>(1, 1) = 1;
    roll_matrix.at<float>(1, 2) = 0;
    roll_matrix.at<float>(2, 1) = 0;

    yaw_matrix.at<float>(0, 2) = 0;
    yaw_matrix.at<float>(1, 2) = 0;
    yaw_matrix.at<float>(2, 0) = 0;
    yaw_matrix.at<float>(2, 1) = 0;
    yaw_matrix.at<float>(2, 2) = 1;

    cv::Mat current_rotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);

    pitch_matrix.at<float>(1, 1) = cos(pitch);
    pitch_matrix.at<float>(1, 2) = -sin(pitch);
    pitch_matrix.at<float>(2, 1) = sin(pitch);
    pitch_matrix.at<float>(2, 2) = cos(pitch);

    roll_matrix.at<float>(0, 0) = cos(roll);
    roll_matrix.at<float>(0, 2) = -sin(roll);
    roll_matrix.at<float>(2, 0) = sin(roll);
    roll_matrix.at<float>(2, 2) = cos(roll);

    yaw_matrix.at<float>(0, 0) = cos(yaw);
    yaw_matrix.at<float>(0, 1) = sin(yaw);
    yaw_matrix.at<float>(1, 0) = -sin(yaw);
    yaw_matrix.at<float>(2, 1) = cos(yaw);

    current_rotation_matrix = yaw_matrix * pitch_matrix * roll_matrix;
    rotation_matrix = rotation_matrix * current_rotation_matrix;
}