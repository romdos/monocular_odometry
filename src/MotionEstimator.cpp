#include "MotionEstimator.h"

MotionEstimator::MotionEstimator(const char* parameters)
{
    /* Read parameters */
    std::string line;
    try
    {
        std::ifstream file(parameters);
        std::getline(file, line);
        focus_ = atof(line.substr(line.find_first_of(".0123456789")).c_str());
        std::getline(file, line);
        pixel_size_ = atof(line.substr(line.find_first_of(".0123456789")).c_str());
        std::getline(file, line);
        height_ = atof(line.substr(line.find_first_of(".0123456789")).c_str());
    }
    catch (std::out_of_range &e)
    {
        std::cerr << "Could not file with parameters. Replace path to the directory with your own.\n";
    };

    sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &MotionEstimator::processFrame, this);
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/transform", 5);

    rotation_matrix_ = cv::Mat::eye(3, 3, CV_32FC1);

    camera_pose_.pose.position.x = 0;
    camera_pose_.pose.position.y = 0;
    camera_pose_.pose.position.z = 0;

    camera_pose_.header.frame_id = "camera_frame";
    time_now_ = ros::Time::now().toSec();
}

void MotionEstimator::processFrame(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // todo: read grayscale
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    findCameraPose(cv_ptr, 0.4);
    pub_.publish(camera_pose_);
}

void MotionEstimator::findCameraPose(cv_bridge::CvImagePtr& cv_ptr, float time_threshold)
{
    cv::Mat frame = cv_ptr -> image;

    /* Gradient estimation via Sobel operator */
    int scale = 1;
    int delta = 0;
    int ddepth = CV_32FC1;

    if (cv_ptr -> header.stamp.toSec() - time_now_ < time_threshold) // todo: adjust
    {
        previous_frame_ = frame;

        cvtColor(previous_frame_, previous_frame_, CV_BGR2GRAY);
        previous_frame_.convertTo(previous_frame_, CV_32FC1, 1.0 / 255.0);

        Sobel(previous_frame_, grad_x_, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
        Sobel(previous_frame_, grad_y_, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
        printf("%g\n", cv_ptr -> header.stamp.toSec() - time_now_);

        return;
    }

    cv::Mat norm_frame;

    cvtColor(frame, frame, CV_BGR2GRAY);
    frame.convertTo(norm_frame, CV_32FC1, 1.0 / 255.0);

    cv::Mat diff = cv::Mat::zeros(frame.rows, frame.cols, CV_32FC1);

    cv::absdiff(norm_frame, previous_frame_, diff);
    /* Least Squares Method for transform parameters finding */
    cv::Mat_<double> transform_params = least_squares_method(diff);
    /* Translation vector */

    cv::Vec3d current_translation(transform_params.at<double>(0, 0),
                                  transform_params.at<double>(1, 0),
                                  transform_params.at<double>(2, 0));

    /* Rotation */
    updateRotationMatrix(transform_params.at<double>(3, 0),
                         transform_params.at<double>(4, 0),
                         transform_params.at<double>(5, 0));

    cv::Mat_<double> translation = rotation_matrix_ * cv::Mat(current_translation);
    camera_pose_.pose.position.x += translation(0, 0);
    camera_pose_.pose.position.y += translation(1, 0);
    camera_pose_.pose.position.z += translation(2, 0);

    // todo: camera orientation -> quaternions!

    previous_frame_ = norm_frame.clone();

    Sobel(previous_frame_, grad_x_, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    Sobel(previous_frame_, grad_y_, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
}

cv::Mat_<double> MotionEstimator::least_squares_method(const cv::Mat & diff)
{
    int image_width  = diff.cols;
    int image_height = diff.rows;

    // todo: sum over ROI!
    auto vertical_level = (int) (1.0 * image_height / 2 + 1);
    int points_number = vertical_level * image_width;

    cv::Mat A = cv::Mat::zeros(points_number, 6, CV_64FC1);
    cv::Mat b = cv::Mat::zeros(points_number, 1, CV_64FC1);

    for (int i = 0; i < vertical_level; i++)
    {
        for (int j = 0; j < image_width; j++)
        {
            double dI = diff.at<float>(i, j); // ~10^-3
            double I_x = grad_x_.at<double>(i, j);
            double I_y = grad_y_.at<double>(i, j); // ~10^-2

            double y_horizont = pixel_size_ * vertical_level;

            double y = y_horizont + pixel_size_ * (i + 1);
            double x = pixel_size_ * j;  // ~10^-2
            double Z = focus_ * height_ / (y - y_horizont);  // ~10^3

            A.at<double>(i * image_width + j, 0) = -focus_ * I_x / Z;
            A.at<double>(i * image_width + j, 1) = -focus_ * I_y / Z;
            A.at<double>(i * image_width + j, 2) = (I_x * x + I_y * y) / Z; // very small
            A.at<double>(i * image_width + j, 3) = I_x * x * y / focus_ + I_y * (focus_ + y * y / focus_);
            A.at<double>(i * image_width + j, 4) = -(focus_ + x * x / focus_) * I_x - I_y * x * y / focus_;
            A.at<double>(i * image_width + j, 5) = I_x * y - I_y * x;

            b.at<double>(i * image_width + j, 0) = dI;
        }
    }
    // todo: why det(A) is equal to zeros???
    std::cout <<  cv::determinant(A.t() * A) << std::endl;

    return (A.t() * A).inv() * A.t() * b;
}

void MotionEstimator::updateRotationMatrix(const float pitch, const float roll, const float yaw)
{
    /* Transform matrix construction */
    cv::Mat pitch_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat roll_matrix  = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat yaw_matrix   = cv::Mat::zeros(3, 3, CV_32FC1);

    pitch_matrix.at<double>(0, 0) = 1;
    pitch_matrix.at<double>(0, 1) = 0;
    pitch_matrix.at<double>(0, 2) = 0;
    pitch_matrix.at<double>(1, 0) = 0;
    pitch_matrix.at<double>(2, 0) = 0;

    roll_matrix.at<double>(0, 1) = 0;
    roll_matrix.at<double>(1, 0) = 0;
    roll_matrix.at<double>(1, 1) = 1;
    roll_matrix.at<double>(1, 2) = 0;
    roll_matrix.at<double>(2, 1) = 0;

    yaw_matrix.at<double>(0, 2) = 0;
    yaw_matrix.at<double>(1, 2) = 0;
    yaw_matrix.at<double>(2, 0) = 0;
    yaw_matrix.at<double>(2, 1) = 0;
    yaw_matrix.at<double>(2, 2) = 1;

    cv::Mat current_rotation_matrix = cv::Mat::zeros(3, 3, CV_32FC1);

    pitch_matrix.at<double>(1, 1) = cos(pitch);
    pitch_matrix.at<double>(1, 2) = -sin(pitch);
    pitch_matrix.at<double>(2, 1) = sin(pitch);
    pitch_matrix.at<double>(2, 2) = cos(pitch);

    roll_matrix.at<double>(0, 0) = cos(roll);
    roll_matrix.at<double>(0, 2) = -sin(roll);
    roll_matrix.at<double>(2, 0) = sin(roll);
    roll_matrix.at<double>(2, 2) = cos(roll);

    yaw_matrix.at<double>(0, 0) = cos(yaw);
    yaw_matrix.at<double>(0, 1) = sin(yaw);
    yaw_matrix.at<double>(1, 0) = -sin(yaw);
    yaw_matrix.at<double>(2, 1) = cos(yaw);

    current_rotation_matrix = yaw_matrix * (pitch_matrix * roll_matrix);
    rotation_matrix_ = rotation_matrix_ * current_rotation_matrix;
}
