//
// Created by wb on 2020/8/10.
//

#include "LidarMotionUndistortion.h"
#include <iostream>

pcl::visualization::CloudViewer g_PointCloudView("PointCloud View");

LidarMotionCalibrator::LidarMotionCalibrator(tf::TransformListener* tf)
{
    tf_ = tf;
    scan_sub_ = nh_.subscribe("champion_scan", 10, &LidarMotionCalibrator::ScanCallBack, this);
}

LidarMotionCalibrator::~LidarMotionCalibrator()
{
    if(tf_!=NULL)
        delete tf_;
}

// 拿到原始的激光数据来进行处理
void LidarMotionCalibrator::ScanCallBack(const champion_nav_msgs::ChampionNavLaserScanPtr& scan_msg)
{
    //转换到矫正需要的数据
    ros::Time startTime, endTime;
    startTime = scan_msg->header.stamp;

    champion_nav_msgs::ChampionNavLaserScan laserScanMsg = *scan_msg;

    //得到最终点的时间
    int beamNum = laserScanMsg.ranges.size();
    endTime = startTime + ros::Duration(laserScanMsg.time_increment * beamNum);

    // 将数据复制出来
    std::vector<double> angles, ranges;
    for(int i = beamNum - 1; i > 0; i--)
    {
        double lidar_dist = laserScanMsg.ranges[i];
        double lidar_angle = laserScanMsg.angles[i];

        if(lidar_dist < 0.05 || std::isnan(lidar_dist) || std::isinf(lidar_dist))
            lidar_dist = 0.0;

        ranges.push_back(lidar_dist);
        angles.push_back(lidar_angle);
    }

    //转换为pcl::pointcloud for visuailization
    tf::Stamped<tf::Pose> visualPose;
    if(!getLaserPose(visualPose, startTime, tf_))
    {
        ROS_WARN("Not visualPose, can not Calib");
        return;
    }

    double visualYaw = tf::getYaw(visualPose.getRotation());

    visual_cloud_.clear();
    for(int i = 0; i < ranges.size();i++)
    {
        if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
            continue;

        double x = ranges[i] * cos(angles[i]);
        double y = ranges[i] * sin(angles[i]);

        pcl::PointXYZRGB pt;
        pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
        pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
        pt.z = 1.0;

        // pack r/g/b into rgb
        unsigned char r = 255, g = 0, b = 0;    //red color
        unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
        pt.rgb = *reinterpret_cast<float*>(&rgb);

        visual_cloud_.push_back(pt);
    }
    std::cout << std::endl;

    //进行矫正
    Lidar_Calibration(ranges, angles,
                      startTime,
                      endTime,
                      tf_);

    //转换为pcl::pointcloud for visuailization
    for(int i = 0; i < ranges.size();i++)
    {
        if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
            continue;

        double x = ranges[i] * cos(angles[i]);
        double y = ranges[i] * sin(angles[i]);

        pcl::PointXYZRGB pt;
        pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
        pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
        pt.z = 1.0;

        unsigned char r = 0, g = 255, b = 0;    // green color
        unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
        pt.rgb = *reinterpret_cast<float*>(&rgb);

        visual_cloud_.push_back(pt);
    }

    //进行显示
    g_PointCloudView.showCloud(visual_cloud_.makeShared());
}

/**
 * @name getLaserPose()
 * @brief 得到机器人在里程计坐标系中的位姿tf::Pose
 *        得到dt时刻激光雷达在odom坐标系的位姿
 * @param odom_pos  机器人的位姿
 * @param dt        dt时刻
 * @param tf_
*/
bool LidarMotionCalibrator::getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                  ros::Time dt,
                  tf::TransformListener * tf_)
{
    odom_pose.setIdentity();

    tf::Stamped < tf::Pose > robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "base_laser";
    robot_pose.stamp_ = dt;   //设置为ros::Time()表示返回最近的转换关系

    // get the global pose of the robot
    try
    {
        if(!tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5)))  // 0.15s 的时间可以修改
        {
            ROS_ERROR("LidarMotion-Can not Wait Transform()");
            return false;
        }
        tf_->transformPose("/odom", robot_pose, odom_pose);
    }
    catch (tf::LookupException& ex)
    {
        ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf::ConnectivityException& ex)
    {
        ROS_ERROR("LidarMotion: Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf::ExtrapolationException& ex)
    {
        ROS_ERROR("LidarMotion: Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }

    return true;
}

/**
 * @brief Lidar_MotionCalibration
 *        激光雷达运动畸变去除分段函数;
 *        在此分段函数中，认为机器人是匀速运动；
 * @param frame_base_pose       标定完毕之后的基准坐标系
 * @param frame_start_pose      本分段第一个激光点对应的位姿
 * @param frame_end_pose        本分段最后一个激光点对应的位姿
 * @param ranges                激光数据－－距离
 * @param angles                激光数据－－角度
 * @param startIndex            本分段第一个激光点在激光帧中的下标
 * @param beam_number           本分段的激光点数量
 */
void LidarMotionCalibrator::Lidar_MotionCalibration(
        tf::Stamped<tf::Pose> frame_base_pose,
        tf::Stamped<tf::Pose> frame_start_pose,
        tf::Stamped<tf::Pose> frame_end_pose,
        std::vector<double>& ranges,
        std::vector<double>& angles,
        int startIndex,
        int& beam_number)
{
    //每个位姿进行线性插值时的步长
    double beam_step = 1.0 / (beam_number - 1);

    //机器人的起始角度 和 最终角度
    tf::Quaternion start_angle_q = frame_start_pose.getRotation();
    tf::Quaternion end_angle_q = frame_end_pose.getRotation();

    //转换到弧度
    double start_angle_r = tf::getYaw(start_angle_q);
    double base_angle_r = tf::getYaw(end_angle_q);

    //机器人的起始位姿
    tf::Vector3 start_pos = frame_start_pose.getOrigin();
    start_pos.setZ(0);

    //最终位姿
    tf::Vector3 end_pos = frame_end_pose.getOrigin();
    end_pos.setZ(0);

    //基础坐标系
    tf::Vector3 base_pos = frame_base_pose.getOrigin();
    base_pos.setZ(0);

    double mid_angle;
    tf::Vector3 mid_pos;
    tf::Vector3 mid_point;

    double lidar_angle, lidar_dist;

    //插值计算每个点对应的位姿
    for(int i = 0; i< beam_number; i++)
    {
        //角度插值
        mid_angle = tf::getYaw(start_angle_q.slerp(end_angle_q, beam_step * i));

        //线性插值
        mid_pos = start_pos.lerp(end_pos, beam_step * i);

        //得到激光点在odom 坐标系中的坐标
        double tmp_angle;

        //如果range不等于无穷,则需要进行矫正
        if( tfFuzzyZero(ranges[startIndex + i]) == false )
        {
            //计算对应的激光点在odom坐标系中的坐标
            //得到这帧激光束距离和夹角
            lidar_dist  = ranges[startIndex+i];
            lidar_angle = angles[startIndex+i];

            //激光雷达坐标系下的坐标
            double laser_x, laser_y;
            laser_x = lidar_dist * cos(lidar_angle);
            laser_y = lidar_dist * sin(lidar_angle);

            //里程计坐标系下的坐标
            double odom_x, odom_y;
            odom_x = laser_x * cos(mid_angle) - laser_y * sin(mid_angle) + mid_pos.x();
            odom_y = laser_x * sin(mid_angle) + laser_y * cos(mid_angle) + mid_pos.y();

            //转换到类型中去
            mid_point.setValue(odom_x, odom_y, 0);

            //把在odom坐标系中的激光数据点 转换到 基础坐标系
            double x0, y0, a0, s, c;
            x0 = base_pos.x();
            y0 = base_pos.y();
            a0 = base_angle_r;
            s = sin(a0);
            c = cos(a0);
            /*
             * 把base转换到odom 为[c -s x0
             *                    s  c y0
             *                    0  0 1]
             * 把odom转换到base为 [c  s -x0*c-y0*s
             *                    -s c x0*s-y0*c
             *                    0  0 1]
             */
            double tmp_x,tmp_y;
            tmp_x =  mid_point.x()*c  + mid_point.y()*s - x0*c - y0*s;
            tmp_y = -mid_point.x()*s  + mid_point.y()*c  + x0*s - y0*c;
            mid_point.setValue(tmp_x, tmp_y, 0);

            //然后计算以起始坐标为起点的 dist angle
            double dx,dy;
            dx = (mid_point.x());
            dy = (mid_point.y());
            lidar_dist = sqrt(dx*dx + dy*dy);
            lidar_angle = atan2(dy, dx);

            //激光雷达被矫正
            ranges[startIndex+i] = lidar_dist;
            angles[startIndex+i] = lidar_angle; //基础坐标系下激光的距离和角度值
        }
        //如果等于无穷,则随便计算一下角度
        else
        {
            //激光角度
            lidar_angle = angles[startIndex+i];

            //里程计坐标系的角度
            tmp_angle = mid_angle + lidar_angle;
            tmp_angle = tfNormalizeAngle(tmp_angle);

            //如果数据非法 则只需要设置角度就可以了。把角度换算成start_pos坐标系内的角度
            lidar_angle = tfNormalizeAngle(tmp_angle - start_angle_r);

            angles[startIndex+i] = lidar_angle;
        }
    }
}

/**
 * @name Lidar_Calibration()
 * @brief 激光雷达数据　分段线性进行差值　分段的周期为5ms
 * @param ranges 激光束的距离值集合
 * @param angle　激光束的角度值集合
 * @param startTime　第一束激光的时间戳
 * @param endTime　最后一束激光的时间戳
 * @param *tf_
*/
void LidarMotionCalibrator::Lidar_Calibration(std::vector<double>& ranges,
                       std::vector<double>& angles,
                       ros::Time startTime,
                       ros::Time endTime,
                       tf::TransformListener * tf_)
{
    // 统计激光束的数量
    int beamNumber = ranges.size();
    if(beamNumber != angles.size())
    {
        ROS_ERROR("Error:ranges not match to the angles");
        return;
    }

    // 5ms来进行分段
    int interpolation_time_duration = 5 * 1000;

    tf::Stamped<tf::Pose> frame_start_pose;
    tf::Stamped<tf::Pose> frame_mid_pose;
    tf::Stamped<tf::Pose> frame_base_pose;
    tf::Stamped<tf::Pose> frame_end_pose;

    // 起始时间，单位为us
    double start_time = startTime.toSec() * 1000 * 1000;
    double end_time = endTime.toSec() * 1000 * 1000;
    double time_inc = (end_time - start_time) / beamNumber; // 每束激光数据的时间间隔

    // 当前插值的段的起始坐标
    int start_index = 0;

    // 起始点的位姿，这里要得到起始点位置的原因是：起始点就是我们的base_pose
    // 所有的激光点的基准位姿都会改成base_pose
    // ROS_INFO("get start pose");

    if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
    {
        ROS_WARN("Not Start Pose,Can not Calib");
        return ;
    }

    if(!getLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0),tf_))
    {
        ROS_WARN("Not End Pose, Can not Calib");
        return ;
    }

    int cnt = 0;
    //基准坐标就是第一个位姿的坐标
    frame_base_pose = frame_start_pose;
    for(int i = 0; i < beamNumber; i++)
    {
        //分段线性,时间段的大小为interpolation_time_duration
        double mid_time = start_time + time_inc * (i - start_index);
        if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
        {
            cnt++;

            if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0), tf_))
            {
                ROS_ERROR("Mid %d Pose Error", cnt);
                return ;
            }

            //对当前的起点和终点进行插值
            //interpolation_time_duration中间有多少个点.
            int interp_count = i - start_index + 1;

            Lidar_MotionCalibration(frame_base_pose,
                                    frame_start_pose,
                                    frame_mid_pose,
                                    ranges,
                                    angles,
                                    start_index,
                                    interp_count);

            //更新时间
            start_time = mid_time;
            start_index = i;
            frame_start_pose = frame_mid_pose;
        }
    }
}