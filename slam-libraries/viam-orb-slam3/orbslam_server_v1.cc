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
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <cfenv>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <sstream>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include "SlamService.h"
#include "Utils.h"

#include "proto/api/common/v1/common.grpc.pb.h"
#include "proto/api/common/v1/common.pb.h"
#include "proto/api/service/slam/v1/slam.grpc.pb.h"
#include "proto/api/service/slam/v1/slam.pb.h"
#pragma STDC FENV_ACCESS ON
#define _USE_MATH_DEFINES
using namespace std;
using namespace boost::filesystem;
#define IMAGE_SIZE 300
#define CHECK_FOR_SHUTDOWN_INTERVAL 1e5
#define MAX_COLOR_VALUE 255
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;

using proto::api::common::v1::PointCloudObject;
using proto::api::common::v1::Pose;
using proto::api::common::v1::PoseInFrame;
using proto::api::service::slam::v1::GetMapRequest;
using proto::api::service::slam::v1::GetMapResponse;
using proto::api::service::slam::v1::GetPositionRequest;
using proto::api::service::slam::v1::GetPositionResponse;
using proto::api::service::slam::v1::SLAMService;
using SlamPtr = std::unique_ptr<ORB_SLAM3::System>;

void exit_loop_handler(int s) {
    BOOST_LOG_TRIVIAL(info) << "Finishing session";
    b_continue_session = false;
}

int main(int argc, char **argv) {
    // TODO: change inputs to match args from rdk
    // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-179
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    if (argc < 6) {
        BOOST_LOG_TRIVIAL(fatal) << "No args found. Expected: \n"
                                 << "./bin/orb_grpc_server "
                                    "-data_dir=path_to_data "
                                    "-config_param={mode=slam_mode,} "
                                    "-port=grpc_port "
                                    "-sensors=sensor_name "
                                    "-data_rate_ms=frame_delay"
                                    "-map_rate_sec=map_rate_sec";
        return 1;
    }

    string config_params = viam::utils::argParser(argc, argv, "-config_param=");

    const auto debugParam = viam::utils::configMapParser(config_params, "debug=");
    bool isDebugTrue;
    bool isDebugOne;
    istringstream(debugParam) >> std::boolalpha >> isDebugTrue;
    istringstream(debugParam) >> std::noboolalpha >> isDebugOne;
    if (!isDebugTrue && !isDebugOne) {
        boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                            boost::log::trivial::info);
    }

    for (int i = 0; i < argc; ++i) {
        BOOST_LOG_TRIVIAL(debug) << "Argument #" << i << " is " << argv[i];
    }
    // setup the SLAM server
    viam::SLAMServiceImpl slamService;
    ServerBuilder builder;

    string actual_path = viam::utils::argParser(argc, argv, "-data_dir=");
    if (actual_path.empty()) {
        BOOST_LOG_TRIVIAL(fatal) << "No data directory given";
        return 1;
    }
    string path_to_vocab = actual_path + "/config/ORBvoc.txt";
    string path_to_settings = actual_path + "/config";  // testORB.yaml";

    slamService.path_to_data =
        actual_path + "/data";  // will change in DATA 127/181

    // leaving commented for possible testing
    string dummyPath = "/home/kkufieta/slam/slam-libraries/viam-orb-slam3/";
    slamService.path_to_data = dummyPath + "/data_outer/data";
    slamService.path_to_sequence = "Out_file.txt";
    string slam_mode = viam::utils::configMapParser(config_params, "mode=");
    if (slam_mode.empty()) {
        BOOST_LOG_TRIVIAL(fatal) << "No SLAM mode given";
        return 1;
    }

    string slam_port = viam::utils::argParser(argc, argv, "-port=");
    if (slam_port.empty()) {
        BOOST_LOG_TRIVIAL(fatal) << "No gRPC port given";
        return 1;
    }

    string frames = viam::utils::argParser(argc, argv, "-data_rate_ms=");
    if (frames.empty()) {
        BOOST_LOG_TRIVIAL(fatal) << "No camera data rate specified";
        return 1;
    }
    slamService.frame_delay = stoi(frames);

    string map_rate_sec = viam::utils::argParser(argc, argv, "-map_rate_sec=");
    if (map_rate_sec.empty()) {
        BOOST_LOG_TRIVIAL(fatal) << "No map capture rate specified";
        return 1;
    }
    slamService.map_rate_sec = stoi(map_rate_sec);

    slamService.camera_name = viam::utils::argParser(argc, argv, "-sensors=");
    if (slamService.camera_name.empty()) {
        BOOST_LOG_TRIVIAL(info) << "No camera given -> running in offline mode";
        slamService.offlineFlag = true;
    }

    builder.AddListeningPort(slam_port, grpc::InsecureServerCredentials());
    builder.RegisterService(&slamService);

    // Start the SLAM gRPC server
    std::unique_ptr<Server> server(builder.BuildAndStart());
    BOOST_LOG_TRIVIAL(info) << "Server listening on " << slam_port.c_str();

    // Determine which settings file to use(.yaml)
    const path myPath(path_to_settings);
    path latest;
    std::time_t latest_tm = 0;
    for (auto &&entry :
         boost::make_iterator_range(directory_iterator(myPath), {})) {
        path p = entry.path();

        if (is_regular_file(p) && p.extension() == ".yaml") {
            std::time_t timestamp = last_write_time(p);
            if (timestamp > latest_tm) {
                if (slamService.offlineFlag ||
                    p.stem().string().find(slamService.camera_name) !=
                        string::npos) {
                    latest = p;
                    latest_tm = timestamp;
                }
            }
        }
    }
    if (latest.empty()) {
        BOOST_LOG_TRIVIAL(fatal)
            << "No correctly formatted .yaml file found, Expected:\n"
               "{sensor}_data_{dateformat}.yaml";
        return 1;
    }

    // report the current yaml file check if it matches our format
    const string myYAML = latest.stem().string();
    BOOST_LOG_TRIVIAL(debug) << "Our yaml file: " << myYAML;
    string full_path_to_settings =
        path_to_settings + "/" + latest.filename().string();
    if (slamService.offlineFlag) {
        if (myYAML.find("_data_") != string::npos)
            slamService.camera_name = myYAML.substr(0, myYAML.find("_data_"));
        else {
            BOOST_LOG_TRIVIAL(fatal)
                << "No correctly formatted .yaml file found, Expected:\n"
                   "{sensor}_data_{dateformat}.yaml\n"
                   "as most the recent config in directory";
            return 1;
        }
    }

    // Grab timestamp from yaml
    slamService.yamlTime = viam::utils::readTimeFromFilename(
        myYAML.substr(myYAML.find("_data_") + FILENAME_CONST));
    BOOST_LOG_TRIVIAL(debug)
        << "The time from our config is: " << slamService.yamlTime
        << " seconds";

    // Start SLAM
    SlamPtr SLAM = nullptr;
    boost::algorithm::to_lower(slam_mode);

    if (slam_mode == "rgbd") {
        BOOST_LOG_TRIVIAL(info) << "RGBD selected";

        // Create SLAM system. It initializes all system threads and gets ready
        // to process frames.
        SLAM = std::make_unique<ORB_SLAM3::System>(
            path_to_vocab, full_path_to_settings, ORB_SLAM3::System::RGBD,
            false, 0);
        if (slamService.offlineFlag) {
            BOOST_LOG_TRIVIAL(info) << "Running in offline mode";
            // slamService.process_rgbd_offline(SLAM.get());
            slamService.process_rgbd_old(SLAM);
            // Continue to serve requests.
            while (b_continue_session) {
                usleep(CHECK_FOR_SHUTDOWN_INTERVAL);
            }
        } else {
            BOOST_LOG_TRIVIAL(info) << "Running in online mode";
            // slamService.process_rgbd_online(SLAM.get());
        }
        // slamService.process_rgbd_old(SLAM);
        // slamService.process_rgbd_for_testing(SLAM.get());

    } else if (slam_mode == "mono") {
        // TODO implement MONO
        // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-182
    } else {
        BOOST_LOG_TRIVIAL(fatal) << "Invalid slam_mode=" << slam_mode;
        return 1;
    }
    // Save the map here - once
    slamService.save_atlas_as_osa_with_timestamp(SLAM.get(), dummyPath);

    SLAM->Shutdown();
    BOOST_LOG_TRIVIAL(info) << "System shutdown";

    return 0;
}
