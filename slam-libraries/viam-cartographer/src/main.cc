// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#include <signal.h>

#include <iostream>
#include <thread>

#include "glog/logging.h"
#include "slam_service/config.h"

void exit_loop_handler(int s) {
    LOG(INFO) << "Finishing session.\n";
    viam::b_continue_session = false;
}

int main(int argc, char** argv) {
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    int err =
        viam::slam_service::config::ParseAndValidateConfigParams(argc, argv);
    if (err != 0) {
        return err;
    }

    while (viam::b_continue_session) {
        LOG(INFO) << "Cartographer is running\n";
        std::this_thread::sleep_for(std::chrono::microseconds(
            viam::checkForShutdownIntervalMicroseconds));
    }
}
