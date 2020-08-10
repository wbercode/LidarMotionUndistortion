//
// Created by wb on 2020/8/10.
//

#include "LidarMotionUndistortion.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LidarMotionCalib");

    tf::TransformListener tf(ros::Duration(10.0));

    LidarMotionCalibrator tmpLidarMotionCalib(&tf);

    ros::spin();
    return 0;
}