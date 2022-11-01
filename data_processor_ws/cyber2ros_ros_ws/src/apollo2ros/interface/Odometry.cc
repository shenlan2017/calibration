#include "Odometry.h"

namespace interface
{
void Odometry::Compute(double &current_timestamp, double *wheel_speed)
{
    double linear_velo  = (wheel_speed[0] + wheel_speed[1]) / 2.0f;
    double angular_velo = (wheel_speed[0] - wheel_speed[1]) / wheelbase;
    UpdateOdometry(current_timestamp, linear_velo, angular_velo);
}

void Odometry::UpdateOdometry(double &current_timestamp, double &speed, double &angle)
{
    if (!init_flag){
        last_timestamp = current_timestamp;
        init_flag = true;
    }else{
        double delta_timestamp = current_timestamp - last_timestamp;
        pose_cx += (speed * cos(angle)) * delta_timestamp;
        pose_cy += (speed * sin(angle)) * delta_timestamp;
        pose_th += angle * delta_timestamp;
        last_timestamp = current_timestamp;
        if (pose_th >= 3.14) pose_th = -3.14;
        else if (pose_th <= -3.14) pose_th = 3.14;
    }
}

void Odometry::GetOdometry(nav_msgs::Odometry &odom)
{
    odom.pose.pose.position.x = pose_cx;
    odom.pose.pose.position.y = pose_cy;
    odom.pose.pose.position.z = 0.0f;
    // geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(pose_th);
    // odom.pose.pose.orientation = q;

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, pose_th);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q, q_msg);
    odom.pose.pose.orientation = q_msg;
}

}
