//
// Created by wb on 2020/8/10.
//

#ifndef LASERUNDISTORTION_WS_LIDARMOTIONUNDISTORTION_H
#define LASERUNDISTORTION_WS_LIDARMOTIONUNDISTORTION_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>

#include <sensor_msgs/LaserScan.h>

#include <champion_nav_msgs/ChampionNavLaserScan.h>


class LidarMotionCalibrator
{
public:
    LidarMotionCalibrator(tf::TransformListener* tf);

    ~LidarMotionCalibrator();

    void ScanCallBack(const champion_nav_msgs::ChampionNavLaserScanPtr& scan_msg);

    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                      ros::Time dt,
                      tf::TransformListener * tf_);

    void Lidar_MotionCalibration(
            tf::Stamped<tf::Pose> frame_base_pose,
            tf::Stamped<tf::Pose> frame_start_pose,
            tf::Stamped<tf::Pose> frame_end_pose,
            std::vector<double>& ranges,
            std::vector<double>& angles,
            int startIndex,
            int& beam_number);

    void Lidar_Calibration(std::vector<double>& ranges,
                           std::vector<double>& angles,
                           ros::Time startTime,
                           ros::Time endTime,
                           tf::TransformListener * tf_);

public:
    tf::TransformListener* tf_;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;

    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
};

#endif //LASERUNDISTORTION_WS_LIDARMOTIONUNDISTORTION_H
