#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <dirent.h>

#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "../common/logger.hpp"

namespace interface
{

class DataReader{
public:
    // 读取文件夹中的所有文件夹和文件名
    // mode=0:不允许递归, 输出文件和文件夹; mode=1:允许递归, 但是只输出文件名; mode=2:允许递归, 但是只输出文件夹; mode=3: 允许递归, 输出文件名和文件路径;
    static void ReadFolder(const std::string &folder, std::vector<std::string> &files, int mode = 3);

    // (推荐)拷贝文件或者文件夹: 如果源为普通文件, 目标为文件夹形式, 则会自动创建同名普通文件, 如果目标为普通文件, 则会覆盖目标文件. 模式0表示拷贝不改变源文件, 模式1表示移动文件
    static bool CopyFiles(const std::string &src, const std::string &dst, int mode = 0);

    // 拷贝文件夹, 支持递归拷贝
    static bool CopyDirectory(const std::string &strSourceDir, const std::string &strDestDir);

    // 读取YAML文件格式的传感器外参
    static void LoadExtrinsic(const std::string &file_path, Eigen::Affine3d &extrinsic);

    // 读取YAML文件格式的传感器内参
    static bool LoadIntrinsic(const std::string &intrinsics_path, cv::Mat&dist_coeffs, cv::Mat &intrisic_mat);

    // 获取文件地址中的文件名
    static bool GetFileNameInPath(const std::string &path, std::string &filename);

    // 获取程序的当前目录
    static std::string GetCurrentDir();
};


} // namespace interface


