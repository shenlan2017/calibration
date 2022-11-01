#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <interpreter.hpp>
#include "interface/DataReader.h"
#include "common/logger.hpp"
#include "common/time_conversion.hpp"
#include "config/config.h"

#define M_PI 3.14159265358979323846   /* pi */
#define M_PI_2 1.57079632679489661923 /* pi/2 */

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "apollo2ros");
    ros::NodeHandle nh;
    common::Logger logger(argc, argv);

    std::string current_ws = ros::package::getPath("apollo2ros");
    // std::string current_ws = interface::DataReader::GetCurrentDir();
    AINFO << "Current Dir is: " << current_ws;
    ezcfg::Interpreter itp(current_ws + "/config/config.txt", true);
    Config conf;
    itp.parse(conf);

    rosbag::Bag target;
    target.open(current_ws + "/" + conf.generate_rosbag_name, rosbag::bagmode::Write);
    std::vector<std::string> topic_folder;
    std::string dag_folder_path = conf.dag_folder_path;
    interface::DataReader::ReadFolder(dag_folder_path, topic_folder, 0);

    if (conf.camera_topic != "" && std::find(topic_folder.begin(), topic_folder.end(), dag_folder_path + "/image") != topic_folder.end())
    {
        // todo:
    }

    if (conf.rtk_topic != "" && std::find(topic_folder.begin(), topic_folder.end(), dag_folder_path + "/rtk_pose") != topic_folder.end())
    {
        std::deque<std::vector<double>> ins_status;
        if (std::find(topic_folder.begin(), topic_folder.end(), dag_folder_path + "/ins_stat") != topic_folder.end())
        {
            std::vector<std::string> stat_files;
            interface::DataReader::ReadFolder(dag_folder_path + "/ins_stat", stat_files, 0);
            sort(stat_files.begin(), stat_files.end());
            for (auto &f : stat_files)
            {
                std::ifstream file_stream(f, std::ios::in);
                std::string drop_line, line;
                getline(file_stream, drop_line);
                getline(file_stream, drop_line);
                getline(file_stream, line);
                if (!line.empty())
                {
                    std::stringstream ss;
                    ss << line;
                    std::vector<double> input(3, 0.0);
                    ss >> input[0];
                    ss >> input[1];
                    ss >> input[2];
                    ins_status.emplace_back(input);
                }
                file_stream.close();
            }
        }
        else
        {
            AWARN << "FAIL TO GET INS STATUS!";
        }
        std::vector<std::string> rtk_files;
        interface::DataReader::ReadFolder(dag_folder_path + "/rtk_pose", rtk_files, 0);
        sort(rtk_files.begin(), rtk_files.end());
        if (ins_status.size() < rtk_files.size())
        {
            AERROR << "FAIL TO MATCH INS STATUS!";
            return -1;
        }
        // bool init_flag = false;
        // float init_position_x = 0.0f;
        // float init_position_y = 0.0f;
        int count = 0;
        for (auto &f : rtk_files)
        {
            std::ifstream file_stream(f, std::ios::in);
            std::string drop_line, line;
            getline(file_stream, drop_line);
            getline(file_stream, drop_line);
            getline(file_stream, line);
            if (!line.empty())
            {
                std::stringstream ss;
                ss << line;
                nav_msgs::Odometry rtk_msg;
                double measurement_time = 0.0f;
                ss >> measurement_time;
                rtk_msg.header.stamp = ros::Time(measurement_time);
                rtk_msg.header.frame_id = "rtk";
                ss >> rtk_msg.pose.pose.position.x;
                ss >> rtk_msg.pose.pose.position.y;
                ss >> rtk_msg.pose.pose.position.z;
                // if (!init_flag){
                //     init_flag = true;
                //     init_position_x = rtk_msg.pose.pose.position.x;
                //     init_position_y = rtk_msg.pose.pose.position.y;
                // }else{
                //     rtk_msg.pose.pose.position.x -= init_position_x;
                //     rtk_msg.pose.pose.position.y -= init_position_y;
                // }
                // AINFO << rtk_msg.pose.pose.position.x;
                // AINFO << rtk_msg.pose.pose.position.y;
                ss >> rtk_msg.pose.pose.orientation.x;
                ss >> rtk_msg.pose.pose.orientation.y;
                ss >> rtk_msg.pose.pose.orientation.z;
                ss >> rtk_msg.pose.pose.orientation.w;
                ss >> rtk_msg.twist.twist.linear.x;
                ss >> rtk_msg.twist.twist.linear.y;
                ss >> rtk_msg.twist.twist.linear.z;
                if (ins_status.size() != 0 && std::abs(measurement_time - ins_status[count][0]) < 0.02)
                {
                    // std::vector<double> cov(36, 0.0f);
                    // cov[0] = ins_status[count][1];
                    // cov[1] = ins_status[count][2];
                    rtk_msg.pose.covariance[0] = ins_status[count][1];
                    rtk_msg.pose.covariance[1] = ins_status[count][2];
                }
                else if (ins_status.size() != 0)
                {
                    AINFO << std::setprecision(16) << measurement_time;
                    AINFO << std::setprecision(16) << ins_status[count][0];
                    AERROR << "FAIL TO MATCH INS STATUS!";
                    return -1;
                }
                // AINFO << rtk_msg.header.stamp;
                target.write(conf.rtk_topic, rtk_msg.header.stamp, rtk_msg);
            }
            file_stream.close();
            count++;
        }
    }

    if (conf.imu_topic != "" && std::find(topic_folder.begin(), topic_folder.end(), dag_folder_path + "/imu") != topic_folder.end())
    {
        std::vector<std::string> imu_files;
        interface::DataReader::ReadFolder(dag_folder_path + "/imu", imu_files, 0);
        sort(imu_files.begin(), imu_files.end());
        for (auto &f : imu_files){
            std::ifstream file_stream(f, std::ios::in);
            std::string drop_line, line;
            getline(file_stream, drop_line);
            getline(file_stream, drop_line);
            getline(file_stream, line);
            if (!line.empty())
            {
                std::stringstream ss;
                ss << line;
                sensor_msgs::Imu imu_msg;
                double drop_info = 0.0f;
                ss >> drop_info;
                double meas_gps_time = 0.0f;
                ss >> meas_gps_time;
                double meas_utc_time = common::gps2unix(meas_gps_time);
                imu_msg.header.stamp = ros::Time(meas_utc_time);
                imu_msg.header.frame_id = "imu";
                ss >> imu_msg.linear_acceleration.x;
                ss >> imu_msg.linear_acceleration.y;
                ss >> imu_msg.linear_acceleration.z;
                ss >> imu_msg.angular_velocity.x;
                ss >> imu_msg.angular_velocity.y;
                ss >> imu_msg.angular_velocity.z;
                // AINFO << imu_msg.header.stamp;
                target.write(conf.imu_topic, imu_msg.header.stamp, imu_msg);
            }
            file_stream.close();
        }
    }

    if (conf.lidar_topic != "" && std::find(topic_folder.begin(), topic_folder.end(), dag_folder_path + "/lidar_pcd") != topic_folder.end())
    {
        std::vector<std::string> lidar_files;
        interface::DataReader::ReadFolder(dag_folder_path + "/lidar_pcd", lidar_files, 0);
        sort(lidar_files.begin(), lidar_files.end());
        for (auto &f : lidar_files)
        {
            sensor_msgs::PointCloud cloud;
            std::string str_timestamps;
            interface::DataReader::GetFileNameInPath(f, str_timestamps);
            // AINFO << str_timestamps;
            double num_timestamps = stod(str_timestamps);
            cloud.header.stamp = ros::Time(num_timestamps);
            cloud.header.frame_id = "lidar";
            cloud.channels.resize(1);
            cloud.channels[0].name = "intensities";

            std::ifstream file_stream(f, std::ios::in);
            while (!file_stream.eof())
            {
                std::string line;
                std::getline(file_stream, line);
                // cloud.channels[0].values.resize(num_points);
                if (!line.empty())
                {
                    std::stringstream ss;
                    ss << line;
                    std::vector<float> data_buff;
                    std::string temp;
                    while (getline(ss, temp, ',')){
                        data_buff.emplace_back(std::stod(temp));
                    }

                    geometry_msgs::Point32 pt;
                    pt.x = data_buff[0];
                    pt.y = data_buff[1];
                    pt.z = data_buff[2];
                    cloud.points.emplace_back(pt);
                    cloud.channels[0].values.emplace_back(data_buff[3]);
                }
            }
            file_stream.close();

            sensor_msgs::PointCloud2 cloud2;
            cloud2.header.stamp = ros::Time(num_timestamps);
            cloud2.header.frame_id = "lidar";
            convertPointCloudToPointCloud2(cloud, cloud2);
            target.write(conf.lidar_topic, cloud2.header.stamp, cloud2);
        }
    }

    if (conf.odom_topic != "" && std::find(topic_folder.begin(), topic_folder.end(), dag_folder_path + "/chassis") != topic_folder.end())
    {
        std::vector<std::string> chassis_files;
        interface::DataReader::ReadFolder(dag_folder_path + "/chassis", chassis_files, 0);
        sort(chassis_files.begin(), chassis_files.end());
        constexpr double wheel_speed_coeff = 1;
        constexpr double dur_time = 0.01;
        for (auto &f : chassis_files)
        {
            std::ifstream file_stream(f, std::ios::in);
            std::string drop_line, line;
            getline(file_stream, drop_line);
            getline(file_stream, drop_line);
            getline(file_stream, line);
            if (!line.empty())
            {
                std::stringstream ss;
                ss << line;
                geometry_msgs::TwistStamped chassis_msg;
                double measurement_time = 0.0f;
                ss >> measurement_time;
                chassis_msg.header.stamp = ros::Time(measurement_time);
                chassis_msg.header.frame_id = "rtk";
                double tyre_speed[4];
                double speed_mps, steering;
                ss >> tyre_speed[0];
                ss >> tyre_speed[1];
                ss >> tyre_speed[2];
                ss >> tyre_speed[3];
                ss >> speed_mps;
                ss >> steering;
                double vehicle_speed = wheel_speed_coeff * speed_mps;
                double vehicle_angle = 0.0f;
                static constexpr double kEpsilon = 1e-6;
                if (std::fabs(tyre_speed[0]) > kEpsilon)
                {
                    double kappa = tyre_speed[1] / tyre_speed[0];
                    vehicle_angle = 2.0 * (kappa - 1) / (kappa + 1) * vehicle_speed / 1.465;
                }
                chassis_msg.twist.linear.x = vehicle_speed;
                chassis_msg.twist.linear.y = 0.0f;
                chassis_msg.twist.linear.z = 0.0f;
                chassis_msg.twist.angular.x = 0.0f;
                chassis_msg.twist.angular.y = 0.0f;
                chassis_msg.twist.angular.z = vehicle_angle;
                target.write(conf.odom_topic, chassis_msg.header.stamp, chassis_msg);
            }
            file_stream.close();
        }
    }

    if (conf.gnss_topic != "" && std::find(topic_folder.begin(), topic_folder.end(), dag_folder_path + "/best_pose") != topic_folder.end())
    {
        std::vector<std::string> gnss_files;
        interface::DataReader::ReadFolder(dag_folder_path + "/best_pose", gnss_files, 0);
        sort(gnss_files.begin(), gnss_files.end());
        for (auto &f : gnss_files)
        {
            std::ifstream file_stream(f, std::ios::in);
            std::string drop_line, line;
            getline(file_stream, drop_line);
            getline(file_stream, drop_line);
            getline(file_stream, line);
            if (!line.empty())
            {
                std::stringstream ss;
                ss << line;
                sensor_msgs::NavSatFix gnss_msg;
                // todo:实际用的是cyber时间，imu测量是gps时间，需要转换
                double drop_info = 0.0f;
                ss >> drop_info;
                double meas_gps_time = 0.0f;
                ss >> meas_gps_time;
                double meas_utc_time = common::gps2unix(meas_gps_time);
                gnss_msg.header.stamp = ros::Time(meas_utc_time);
                gnss_msg.header.frame_id = "gnss";
                ss >> gnss_msg.latitude;
                ss >> gnss_msg.longitude;
                ss >> gnss_msg.altitude;
                ss >> gnss_msg.position_covariance[0];
                ss >> gnss_msg.position_covariance[3];
                ss >> gnss_msg.position_covariance[6];
                // AINFO << gnss_msg.header.stamp;
                target.write(conf.gnss_topic, gnss_msg.header.stamp, gnss_msg);
            }
            file_stream.close();
        }
    }

    return 0;
}
