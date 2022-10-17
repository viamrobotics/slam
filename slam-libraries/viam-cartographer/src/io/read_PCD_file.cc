// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#include "../src/io/read_PCD_file.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdio.h>

#include <boost/filesystem.hpp>
#include <ctime>
#include <fstream>  // std::ifstream
#include <iostream>
#include <string>

#include "glog/logging.h"

namespace viam {
namespace io {

namespace fs = boost::filesystem;

cartographer::sensor::TimedPointCloudData
ReadFile::timedPointCloudDataFromPCDBuilder(std::string file_path,
                                            std::string initial_filename) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    cartographer::sensor::TimedPointCloudData timedPCD;
    cartographer::sensor::TimedPointCloud ranges;

    // Open the point cloud file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    auto err = pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *cloud);

    if (err == -1) {
        return timedPCD;
    }

    // KAT NOTE: The file name format for the pcd files is assumed to be, e.g.:
    // rplidar_data_2022-02-05T01_00_20.9874.pcd

    // LEAVING COMMENTED UNTIL READY TO TEST
    // double time_delta = ReadTimeFromFilename(
    //     myYAML.substr(initial_filename.find("_data_") + viam::filenamePrefixLength, 
    //     initial_filename.find(".pcd")));

    int start_pos = initial_filename.find("T") + 1;
    int len_pos =
        initial_filename.find(".pcd") - initial_filename.find("T") - 1;
    std::string initial_file = initial_filename.substr(start_pos, len_pos);
    std::string next_file = file_path.substr(start_pos, len_pos);

    std::string::size_type sz;

    // Hour
    float hour_f = std::stof(next_file.substr(0, 2), &sz);
    float hour_i = std::stof(initial_file.substr(0, 2), &sz);

    // Minute
    float min_f = std::stof(next_file.substr(3, 2), &sz);
    float min_i = std::stof(initial_file.substr(3, 2), &sz);

    // Second
    float sec_f = std::stof(next_file.substr(6), &sz);
    float sec_i = std::stof(initial_file.substr(6), &sz);

    float time_delta =
        3600 * (hour_f - hour_i) + 60 * (min_f - min_i) + (sec_f - sec_i);

    LOG(INFO) << "------------ FILE DATA -------------\n";
    LOG(INFO) << "Accessing file " << file_path << " ... ";
    LOG(INFO) << "Loaded " << cloud->width * cloud->height << " data points \n";
    LOG(INFO) << "Size " << cloud->points.size() << "\n";
    LOG(INFO) << "TD " << time_delta << "\n";
    LOG(INFO) << "------------------------------------\n";

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cartographer::sensor::TimedRangefinderPoint TimedRP;
        TimedRP.position = Eigen::Vector3f(
            cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        TimedRP.time = 0 - i * 0.0001;

        ranges.push_back(TimedRP);
    }

    timedPCD.time = cartographer::common::FromUniversal(123) +
                    cartographer::common::FromSeconds(double(time_delta));
    timedPCD.origin = Eigen::Vector3f::Zero();
    timedPCD.ranges = ranges;

    return timedPCD;
}

std::vector<std::string> ReadFile::listFilesInDirectory(
    std::string data_directory) {
    std::vector<std::string> file_paths;

    for (const auto& entry : fs::directory_iterator(data_directory)) {
        file_paths.push_back((entry.path()).string());
    }

    sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

int ReadFile::removeFile(std::string file_path) {
    if (remove(file_path.c_str()) != 0) {
        LOG(INFO) << "Error removing file";
        return 0;
    }
    return 1;
}

// Converts UTC time string to a double value.
double ReadFile::ReadTimeFromFilename(string filename) {
    std::string::size_type sz;
    // Create a stream which we will use to parse the string
    std::istringstream ss(filename);

    // Create a tm object to store the parsed date and time.
    std::tm dt = {0};

    // Now we read from buffer using get_time manipulator
    // and formatting the input appropriately.
    ss >> std::get_time(&dt, time_format.c_str());
    time_t thisTime = std::mktime(&dt);
    auto sub_sec_index = filename.find(".");
    if ((sub_sec_index != string::npos)) {
        double sub_sec = (double)std::stof(filename.substr(sub_sec_index), &sz);
        double myTime = (double)thisTime + sub_sec;
        return myTime;
    } else {
        return (double)thisTime;
    }
}

}  // namespace io
}  // namespace viam
