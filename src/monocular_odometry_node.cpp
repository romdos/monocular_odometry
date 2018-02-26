#include "MotionEstimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_estimator");
    ros::NodeHandle nh;

    MotionEstimator motionEstimator(argv[1]);

    ros::spin();
    return 0;
}