// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>
#include <signal.h>

#include <iostream>
#include <thread>

#include "glog/logging.h"
#include "slam_service/config.h"
#include "slam_service/server_functions.h"
#include "slam_service/slam_service.h"

void exit_loop_handler(int s) {
    LOG(INFO) << "Finishing session.\n";
    viam::b_continue_session = false;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging("viam-cartographer");
    google::SetCommandLineOption("GLOG_minloglevel", "2");
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    viam::SLAMServiceImpl slamService;
    int err =
        viam::config::ParseAndValidateConfigParams(argc, argv, slamService);
    if (err != 0) {
        return err;
    }

    // Setup the SLAM gRPC server
    grpc::ServerBuilder builder;

    std::unique_ptr<int> selected_port = std::make_unique<int>(0);
    builder.AddListeningPort(slamService.port,
                             grpc::InsecureServerCredentials(),
                             selected_port.get());
    builder.RegisterService(&slamService);

    // Start the SLAM gRPC server
    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    LOG(INFO) << "Server listening on " << *selected_port << "\n";

    // Start SLAM: Placeholder function with full "offline mode" functionality
    // that will be picked apart with future tickets into separate functions
    // (GetMap, GetPosition, ProcessDataOnline, ProcessDataOffline).
    LOG(INFO) << "Start mapping: offline mode\n";
    slamService.CreateMap();
    LOG(INFO) << "Done mapping: offline mode\n";

    while (viam::b_continue_session) {
        LOG(INFO) << "Cartographer is running\n";
        std::this_thread::sleep_for(std::chrono::microseconds(
            viam::checkForShutdownIntervalMicroseconds));
    }
}
