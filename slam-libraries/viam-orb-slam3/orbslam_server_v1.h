/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */
#include <System.h>
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_context.h>

#include <atomic>

#include "proto/api/common/v1/common.grpc.pb.h"
#include "proto/api/common/v1/common.pb.h"
#include "proto/api/service/slam/v1/slam.grpc.pb.h"
#include "proto/api/service/slam/v1/slam.pb.h"

using grpc::ServerContext;
using proto::api::service::slam::v1::GetMapRequest;
using proto::api::service::slam::v1::GetMapResponse;
using proto::api::service::slam::v1::GetPositionRequest;
using proto::api::service::slam::v1::GetPositionResponse;
using proto::api::service::slam::v1::SLAMService;

namespace viam {

static const int filenamePrefixLength = 6;
static const int checkForShutdownIntervalMicroseconds = 1e5;
static std::atomic<bool> b_continue_session{true};

class SLAMServiceImpl final : public SLAMService::Service {
   public:
    ::grpc::Status GetPosition(ServerContext *context,
                               const GetPositionRequest *request,
                               GetPositionResponse *response) override;

    ::grpc::Status GetMap(ServerContext *context, const GetMapRequest *request,
                          GetMapResponse *response) override;

    void process_rgbd_online(ORB_SLAM3::System *SLAM);

    void process_rgbd_offline(ORB_SLAM3::System *SLAM);

    // Creates a simple map containing a 2x4x8 rectangular prism with the robot
    // in the center, for testing GetMap and GetPosition.
    void process_rgbd_for_testing(ORB_SLAM3::System *SLAM);

    void start_save_atlas_as_osa(ORB_SLAM3::System *SLAM);

    void stop_save_atlas_as_osa();

    string path_to_data;
    string path_to_map;
    string path_to_sequence;
    string camera_name;
    chrono::milliseconds frame_delay_msec;
    chrono::seconds map_rate_sec;
    double yamlTime;
    std::atomic<bool> offlineFlag{false};

   private:
    void save_atlas_as_osa_with_timestamp(ORB_SLAM3::System *SLAM);

    std::atomic<bool> finished_processing_offline{false};
    std::thread *thread_save_atlas_as_osa_with_timestamp;

    std::mutex slam_mutex;
    Sophus::SE3f poseGrpc;
    std::vector<ORB_SLAM3::MapPoint *> currMapPoints;
};

namespace utils {

enum class FileParserMethod { Recent, Closest };

// find a specific input argument from rdk and write the value to a string.
// Returns empty if the argument is not found.
string argParser(int argc, char **argv, const string varName);

// parse a config map for a specific variable name and return the value as a
// string. Returns empty if the variable is not found within the map.
string configMapParser(string map, string varName);

// Converts UTC time string to a double value.
double readTimeFromFilename(const string filename);

std::vector<std::string> listFilesInDirectoryForCamera(
    const std::string data_directory, const std::string extension,
    const std::string camera_name);

bool loadRGBD(std::string path_to_data, std::string filename, cv::Mat &imRGB,
              cv::Mat &imDepth);

// Find the next frame based off the current interest given a directory of
// data and time to search from
int parseDataDir(const std::vector<std::string> &files,
                 FileParserMethod interest, double configTime,
                 double *timeInterest);

int parseBothDataDir(std::string path_to_data,
                     const std::vector<std::string> &filesRGB,
                     FileParserMethod interest, double configTime,
                     double *timeInterest);

}  // namespace utils
}  // namespace viam