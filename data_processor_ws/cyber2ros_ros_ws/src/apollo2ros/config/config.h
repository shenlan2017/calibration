#pragma once
#include <string>
#include <vector>

struct Config
{
    std::string generate_rosbag_name;
    std::string dag_folder_path;
    std::string camera_topic;
    std::string imu_topic;
    std::string lidar_topic;
    std::string odom_topic;
    std::string gnss_topic;
    std::string rtk_topic;
};
