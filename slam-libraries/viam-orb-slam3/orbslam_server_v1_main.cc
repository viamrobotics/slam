#include "orbslam_server_v1.h"
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>
#include <signal.h>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <chrono>

using namespace boost::filesystem;
using grpc::Server;
using grpc::ServerBuilder;
using SlamPtr = std::unique_ptr<ORB_SLAM3::System>;

void exit_loop_handler(int s) {
    BOOST_LOG_TRIVIAL(info) << "Finishing session";
    viam::b_continue_session = false;
}

int main(int argc, char **argv) {
    // TODO: change inputs to match args from rdk
    // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-179
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    viam::SLAMServiceImpl slamService;

    try {
        const vector<string> args(argv + 1, argv + argc);
        viam::utils::parseAndValidateArguments(args, slamService);
    } catch (const runtime_error &error) {
        BOOST_LOG_TRIVIAL(fatal) << error.what();
        return 1;
    }

    // setup the SLAM server
    ServerBuilder builder;

    std::unique_ptr<int> selected_port = std::make_unique<int>(0);
    builder.AddListeningPort(slamService.slam_port,
                             grpc::InsecureServerCredentials(),
                             selected_port.get());
    builder.RegisterService(&slamService);

    // Start the SLAM gRPC server
    std::unique_ptr<Server> server(builder.BuildAndStart());
    BOOST_LOG_TRIVIAL(info) << "Server listening on " << *selected_port;

    // Determine which settings file to use(.yaml)
    const path myPath(slamService.path_to_settings);
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
        slamService.path_to_settings + "/" + latest.filename().string();
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
        myYAML.substr(myYAML.find("_data_") + viam::filenamePrefixLength));
    BOOST_LOG_TRIVIAL(debug)
        << "The time from our config is: " << slamService.yamlTime
        << " seconds";

    // Start SLAM
    SlamPtr SLAM = nullptr;

    if (slamService.slam_mode == "rgbd") {
        BOOST_LOG_TRIVIAL(info) << "RGBD selected";

        // Create SLAM system. It initializes all system threads and gets ready
        // to process frames.
        SLAM = std::make_unique<ORB_SLAM3::System>(
            slamService.path_to_vocab, full_path_to_settings,
            ORB_SLAM3::System::RGBD, false, 0);
        if (slamService.offlineFlag) {
            BOOST_LOG_TRIVIAL(info) << "Running in offline mode";
            slamService.start_save_atlas_as_osa(SLAM.get());
            slamService.process_rgbd_offline(SLAM.get());
            slamService.stop_save_atlas_as_osa();
            // Continue to serve requests.
            while (viam::b_continue_session) {
                this_thread::sleep_for(chrono::microseconds(
                    viam::checkForShutdownIntervalMicroseconds));
            }
        } else {
            BOOST_LOG_TRIVIAL(info) << "Running in online mode";
            slamService.start_save_atlas_as_osa(SLAM.get());
            slamService.process_rgbd_online(SLAM.get());
            slamService.stop_save_atlas_as_osa();
        }
        // slamService.process_rgbd_for_testing(SLAM.get());

    } else if (slamService.slam_mode == "mono") {
        // TODO implement MONO
        // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-182
    }

    SLAM->Shutdown();
    BOOST_LOG_TRIVIAL(info) << "System shutdown";

    return 0;
}