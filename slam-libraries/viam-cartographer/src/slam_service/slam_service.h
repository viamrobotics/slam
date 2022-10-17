#ifndef SLAM_SERVICE_H_
#define SLAM_SERVICE_H_

#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_context.h>

#include <atomic>
#include <string>

#include "../io/draw_trajectories.h"
#include "../io/read_PCD_file.h"
#include "../io/submap_painter.h"
#include "cartographer/mapping/map_builder.h"
#include "common/v1/common.grpc.pb.h"
#include "common/v1/common.pb.h"
#include "service/slam/v1/slam.grpc.pb.h"
#include "service/slam/v1/slam.pb.h"

using grpc::ServerContext;
using viam::service::slam::v1::GetMapRequest;
using viam::service::slam::v1::GetMapResponse;
using viam::service::slam::v1::GetPositionRequest;
using viam::service::slam::v1::GetPositionResponse;
using viam::service::slam::v1::SLAMService;

namespace viam {

static const int checkForShutdownIntervalMicroseconds = 1e5;
static std::atomic<bool> b_continue_session{true};

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};

class SLAMServiceImpl final : public SLAMService::Service {
   public:
    // GetPosition returns the relative position of the robot w.r.t the "origin"
    // of the map, which is the starting point from where the map was initially
    // created.
    ::grpc::Status GetPosition(ServerContext *context,
                               const GetPositionRequest *request,
                               GetPositionResponse *response) override;

    // GetMap returns either an image or a pointcloud, depending on the MIME
    // type requested
    ::grpc::Status GetMap(ServerContext *context, const GetMapRequest *request,
                          GetMapResponse *response) override;

    void CreateMap();

    // TODO
    // void ProcessDataOnline(ORB_SLAM3::System *SLAM);
    // void ProcessDataOffline(ORB_SLAM3::System *SLAM);
    // Creates a simple map containing a 2x4x8 rectangular prism with the robot
    // in the center, for testing GetMap and GetPosition.
    // void ProcessDataForTesting(ORB_SLAM3::System *SLAM);

    std::string data_dir;
    std::string config_params;
    std::string port;
    std::string sensors;
    std::chrono::milliseconds data_rate_ms;
    std::chrono::seconds map_rate_sec;
    std::string slam_mode;

   private:
    int starting_scan_number = 0;
    int picture_print_interval = 1;
    std::string configuration_directory = "../lua_files";
    std::string configuration_mapping_basename = "mapping_new_map.lua";
    std::string configuration_localization_basename = "locating_in_map.lua";
    std::string configuration_update_basename = "updating_a_map.lua";

    std::mutex slam_mutex;
};

}  // namespace viam

#endif  // SLAM_SERVICE_H_