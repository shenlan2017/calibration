#pragma once

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include "../common/logger.hpp"

namespace interface
{

class Odometry
{
private:
    double wheelbase = 0.0f; // 左右轴距
    double tread = 0.0f;     // 前后轮距
    double minimum_turning_radius = 0.0f;
    double maximum_steering_angle = 0.0f;

    double last_timestamp = 0.0f;
    bool init_flag;
    double pose_cx;
    double pose_cy;
    double pose_th;

    nav_msgs::Path path;

public:
    Odometry(double wheelbase_, double tread_) : wheelbase(wheelbase_), tread(tread_)
    {
        pose_cx = pose_cy = pose_th = 0.0f;
        init_flag = false;
    };

    ~Odometry(){};

    void Compute(double &current_timestamp, double *wheel_speed);

    void UpdateOdometry(double &current_timestamp, double &speed, double &angle);

    void GetOdometry(nav_msgs::Odometry &odom);
};

}
