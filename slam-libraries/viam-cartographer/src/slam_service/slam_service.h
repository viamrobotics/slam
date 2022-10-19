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
extern std::atomic<bool> b_continue_session;

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

    // Placeholder function with full "offline mode" functionality that will
    // be picked apart with future tickets into separate functions (GetMap,
    // GetPosition, ProcessDataOnline, ProcessDataOffline). Gives an overview
    // over how mapping + png map image creation + pbstream map saving is
    // started & executed.
    void CreateMap();

    std::string path_to_data;
    std::string path_to_map;
    std::string config_params;
    std::string port;
    std::string camera_name;
    std::chrono::milliseconds data_rate_ms;
    std::chrono::seconds map_rate_sec;
    std::string slam_mode;
    std::atomic<bool> offlineFlag{false};

   private:
    int starting_scan_number = 0;
    int picture_print_interval = 50;
    std::string configuration_directory = "../lua_files";
    std::string configuration_mapping_basename = "mapping_new_map.lua";
    std::string configuration_localization_basename = "locating_in_map.lua";
    std::string configuration_update_basename = "updating_a_map.lua";
};

}  // namespace viam

#endif  // SLAM_SERVICE_H_
