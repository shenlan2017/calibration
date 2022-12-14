#include "DataReader.h"

namespace interface
{
void DataReader::ReadFolder(const std::string &folder, std::vector<std::string> &files, int mode)
{
#if GCC_VERSION >= 90400
    namespace fs = std::filesystem;
#else
    namespace fs = boost::filesystem;
#endif
    const fs::path folder_path(folder);
    if (fs::is_directory(folder_path))
    {
        for (fs::directory_iterator it(folder_path); it != fs::directory_iterator(); ++it)
        {
            const fs::path cur_pos = folder_path / (it->path()).filename();
            // AINFO << "folder: " << cur_pos.string();

            if (fs::is_directory(cur_pos))
            {
                if (mode != 1) files.emplace_back(cur_pos.string());
                if (mode != 0) ReadFolder(cur_pos.string(), files);
            }
            else if (fs::is_regular_file(cur_pos))
            {
                if (mode != 2) files.emplace_back(cur_pos.string());
            }
        }
    }else{
        AERROR << folder << " is not a directory.";
    }
}

void DataReader::LoadExtrinsic(
    const std::string &file_path, Eigen::Affine3d &extrinsic)
{
    YAML::Node config = YAML::LoadFile(file_path);

    if (config["transform"])
    {
        if (config["transform"]["translation"])
        {
            extrinsic.translation()(0) =
                config["transform"]["translation"]["x"].as<double>();
            extrinsic.translation()(1) =
                config["transform"]["translation"]["y"].as<double>();
            extrinsic.translation()(2) =
                config["transform"]["translation"]["z"].as<double>();
            if (config["transform"]["rotation"])
            {
                double qx = config["transform"]["rotation"]["x"].as<double>();
                double qy = config["transform"]["rotation"]["y"].as<double>();
                double qz = config["transform"]["rotation"]["z"].as<double>();
                double qw = config["transform"]["rotation"]["w"].as<double>();
                extrinsic.linear() =
                    Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
                // AINFO << Eigen::Quaterniond(qw, qx, qy, qz).coeffs();
            }
        }
    }
}

bool DataReader::LoadIntrinsic(const std::string &intrinsics_path, cv::Mat &dist_coeffs, cv::Mat &intrisic_mat)
{
#if GCC_VERSION >= 90400
    if (!(std::filesystem::exists(intrinsics_path)))
        return false;
#else
    if (!(boost::filesystem::exists(intrinsics_path)))
        return false;
#endif
    YAML::Node config = YAML::LoadFile(intrinsics_path);

    if (config["K"] && config["D"])
    {
        std::vector<double> K = config["K"].as<std::vector<double>>();
        std::vector<double> D = config["D"].as<std::vector<double>>();
        intrisic_mat = cv::Mat(3, 3, cv::DataType<double>::type);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                intrisic_mat.at<double>(i, j) = K[i * 3 + j];
            }
        }
        dist_coeffs = cv::Mat(5, 1, cv::DataType<double>::type);
        for (int i = 0; i < 5; i++)
        {
            dist_coeffs.at<double>(i) = D[i];
        }
    }

    return true;
}

bool DataReader::CopyFiles(const std::string &src, const std::string &dst, int mode)
{
#if GCC_VERSION >= 90400
    namespace fs = std::filesystem;
    std::error_code ec;
#else
    namespace fs = boost::filesystem;
    boost::system::error_code ec;
#endif
    const fs::path dst_path(dst);
    const fs::path src_path(src);

    // ????????????????????????????????????????????????
    if (fs::is_regular_file(src_path)){
        // dst is not exist
        if (!fs::exists(dst_path))
            fs::create_directories(dst_path.parent_path());

        // case1: src is a file, dst is also a file
        if (fs::is_regular_file(dst_path) || !fs::exists(dst_path))
        {
#if GCC_VERSION >= 90400
            if (mode == 0){
                fs::copy_file(src_path, dst_path, fs::copy_options::overwrite_existing, ec);
            }else{
                fs::rename(src_path, dst_path, ec);
            }

#else
            if (mode == 0){
                fs::copy_file(src_path, dst_path, fs::copy_option::overwrite_if_exists, ec);
            }else{
                fs::rename(src_path, dst_path, ec);
            }
#endif

            if (ec)
            {
                AERROR << "Copy File Failed: " << ec.message();
                return false;
            }
        }
        // case2: src is a file, dst is a directory
        else if (fs::is_directory(dst_path)){
            fs::path dst_file = dst_path / src_path.filename();
#if GCC_VERSION >= 90400
            if (mode == 0){
                fs::copy_file(src_path, dst_file, fs::copy_options::overwrite_existing, ec);
            }else{
                fs::rename(src_path, dst_file, ec);
            }

#else
            if (mode == 0){
                fs::copy_file(src_path, dst_file, fs::copy_option::overwrite_if_exists, ec);
            }else{
                fs::rename(src_path, dst_file, ec);
            }
#endif
            if (ec)
            {
                AERROR << "Copy File Failed: " << ec.message();
                return false;
            }
        }
        // case3: src is a file, dst is neither file nor directory
        else{
            if (ec)
            {
                AERROR << "Copy File Failed: dst is neither file nor directory. ec: " << ec.message();
                return false;
            }
        }
        return true;
    }

    // ??????????????????????????????????????????
    if (fs::is_directory(src_path)){
        if (!fs::exists(dst_path))
        {
            fs::create_directories(dst_path);
        }
        for (fs::directory_iterator it(src_path); it != fs::directory_iterator(); ++it)
        {
            const fs::path newSrc = src_path / (it->path()).filename();
            const fs::path newDst = dst_path / (it->path()).filename();
            // AINFO << "newSrc: " << newSrc.string() << "\n newDst: " << newDst.string();
            if (fs::is_directory(newSrc))
            {
                CopyFiles(newSrc.string(), newDst.string());
            }
            else if (fs::is_regular_file(newSrc))
            {
#if GCC_VERSION >= 90400
                if (mode == 0){
                    fs::copy_file(newSrc, newDst, fs::copy_options::overwrite_existing, ec);
                }else{
                    fs::rename(newSrc, newDst, ec);
                }

#else
                if (mode == 0){
                    fs::copy_file(newSrc, newDst, fs::copy_option::overwrite_if_exists, ec);
                }else{
                    fs::rename(newSrc, newDst, ec);
                }
#endif
                if (ec)
                {
                    AERROR << "Copy File Failed: " << ec.message();
                    return false;
                }
            }
        }
        return true;
    }

    AERROR << "Error: unrecognized file." << src;
    return false;
}

bool DataReader::CopyDirectory(const std::string &strSourceDir, const std::string &strDestDir)
{
#if GCC_VERSION >= 90400
    namespace fs = std::filesystem;
    std::error_code ec;
#else
    namespace fs = boost::filesystem;
    boost::system::error_code ec;
#endif
    // ??????????????????????????????recursive_directory_iterator???????????????????????????
    fs::recursive_directory_iterator end;

    for (fs::recursive_directory_iterator pos(strSourceDir); pos != end; ++pos)
    {
        //??????????????????????????????????????????
        if (fs::is_directory(*pos))
            continue;
        std::string strAppPath = fs::path(*pos).string();
        std::string strRestorePath;
        // replace_first_copy: ???strAppPath?????????strSourceDir?????????
        // ????????????strDestDir??????, ??????????????????????????????????????????????????????
        boost::algorithm::replace_first_copy(std::back_inserter(strRestorePath), strAppPath, strSourceDir, strDestDir);
        if (!fs::exists(fs::path(strRestorePath).parent_path()))
        {
            fs::create_directories(fs::path(strRestorePath).parent_path(), ec);
        }
#if GCC_VERSION >= 90400
        fs::copy_file(strAppPath, strRestorePath, fs::copy_options::overwrite_existing, ec);
#else
        fs::copy_file(strAppPath, strRestorePath, fs::copy_option::overwrite_if_exists, ec);
#endif
    }
    if (ec)
        return false;

    return true;
}

bool DataReader::GetFileNameInPath(const std::string &path, std::string &filename)
{
#if GCC_VERSION >= 90400
    namespace fs = std::filesystem;
#else
    namespace fs = boost::filesystem;
#endif
    const fs::path _path_(path);
    if (fs::is_regular_file(_path_))
    {
        filename = _path_.filename().string();
        std::string extension = _path_.extension().string();
        filename.erase(filename.size() - extension.size());
        return true;
    }
    else
        return false;
}

std::string DataReader::GetCurrentDir()
{
#if GCC_VERSION >= 90400
    auto curr_path = std::filesystem::current_path();
#else
    auto curr_path = boost::filesystem::current_path();
#endif
    return curr_path.string();
}

} // namespace interface
