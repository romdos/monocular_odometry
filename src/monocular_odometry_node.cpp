#include "MotionEstimator.h"

// todo: push onto github

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_estimator");

    MotionEstimator motionEstimator(argv[1]);

    ros::spin();

    return 0;
}