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
#include "../mapping/map_builder.h"
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

enum class SLAMServiceActionMode { MAPPING, LOCALIZING, UPDATING };

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

    // GetActionMode returns the slam action mode from the provided
    // parameters.
    SLAMServiceActionMode GetActionMode();

    // SetUpMapBuilder loads the lua file with default cartographer config parameters
    // depending on the action mode.
    void SetUpMapBuilder();

    // OverwriteMapBuilderParameters overwrites cartographer specific
    // MapBuilder parameters.
    void OverwriteMapBuilderParameters();

    std::string path_to_data;
    std::string path_to_map;
    std::string config_params;
    std::string port;
    std::string camera_name;
    std::chrono::milliseconds data_rate_ms;
    std::chrono::seconds map_rate_sec;
    std::string slam_mode;
    std::atomic<bool> offlineFlag{false};
    mapping::MapBuilder mapBuilder;

    // -- Cartographer specific config params:
    // MAP_BUILDER.pose_graph
    int optimize_every_n_nodes = 3;
    // TRAJECTORY_BUILDER.trajectory_builder_2d.submaps
    int num_range_data = 100;
    // TRAJECTORY_BUILDER.trajectory_builder_2d
    float missing_data_ray_length = 25.0;
    float max_range = 25.0;
    float min_range = 0.2;
    // TRAJECTORY_BUILDER.pure_localization_trimmer
    int max_submaps_to_keep = 3;  // LOCALIZATION only
    // MAP_BUILDER.pose_graph.overlapping_submaps_trimmer_2d
    int fresh_submaps_count = 3;      // UPDATING only
    double min_covered_area = 1.0;    // UPDATING only
    int min_added_submaps_count = 1;  // UPDATING only
    // MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher
    double occupied_space_weight = 20.0;
    double translation_weight = 10.0;
    double rotation_weight = 1.0;

   private:
    int starting_scan_number = 0;
    int picture_print_interval = 50;
    const std::string configuration_directory = "../lua_files";
    const std::string configuration_mapping_basename = "mapping_new_map.lua";
    const std::string configuration_localization_basename = "locating_in_map.lua";
    const std::string configuration_update_basename = "updating_a_map.lua";
};

}  // namespace viam

#endif  // SLAM_SERVICE_H_
