// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#include "read_PCD_file.h"

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

cartographer::sensor::TimedPointCloudData TimedPointCloudDataFromPCDBuilder(
    std::string file_path, double start_time) {
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

    double current_time = ReadTimeFromFilename(file_path.substr(
        file_path.find("_data_") + viam::io::filenamePrefixLength,
        file_path.find(".pcd")));
    double time_delta = current_time - start_time;

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

std::vector<std::string> ListFilesInDirectory(std::string data_directory) {
    std::vector<std::string> file_paths;

    for (const auto& entry : fs::directory_iterator(data_directory)) {
        file_paths.push_back((entry.path()).string());
    }

    sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

int RemoveFile(std::string file_path) {
    if (remove(file_path.c_str()) != 0) {
        LOG(INFO) << "Error removing file";
        return 0;
    }
    return 1;
}

// Converts UTC time string to a double value.
double ReadTimeFromFilename(std::string filename) {
    // TODO: change time format "%Y-%m-%dT%H:%M:%SZ"
    // https://viam.atlassian.net/browse/DATA-638
    std::string time_format = "%Y-%m-%dT%H_%M_%S";
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
    if ((sub_sec_index != std::string::npos)) {
        double sub_sec = (double)std::stof(filename.substr(sub_sec_index), &sz);
        double myTime = (double)thisTime + sub_sec;
        return myTime;
    } else {
        return (double)thisTime;
    }
}

}  // namespace io
}  // namespace viam
