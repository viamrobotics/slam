#include "slam_service.h"

#include <iostream>
#include <string>

#include "../mapping/map_builder.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"
#include "server_functions.h"

namespace viam {
// GetPosition returns the relative position of the robot w.r.t the "origin" of
// the map, which is the starting point from where the map was initially
// created.
::grpc::Status SLAMServiceImpl::GetPosition(ServerContext *context,
                                            const GetPositionRequest *request,
                                            GetPositionResponse *response) {
    std::cout << "GetPosition" << std::endl;
    return grpc::Status::OK;
}

// GetMap returns either an image or a pointcloud, depending on the MIME type
// requested
::grpc::Status SLAMServiceImpl::GetMap(ServerContext *context,
                                       const GetMapRequest *request,
                                       GetMapResponse *response) {
    std::cout << "GetMap" << std::endl;
    return grpc::Status::OK;
}

void SLAMServiceImpl::CreateMap() {
    mapping::MapBuilder mapBuilder;

    // Add configs
    mapBuilder.SetUp(this->configuration_directory,
                     this->configuration_mapping_basename);

    // Build MapBuilder
    mapBuilder.BuildMapBuilder();

    // Build TrajectoryBuilder
    int trajectory_id = mapBuilder.map_builder_->AddTrajectoryBuilder(
        {kRangeSensorId}, mapBuilder.trajectory_builder_options_,
        mapBuilder.GetLocalSlamResultCallback());

    LOG(INFO) << "Trajectory ID: " << trajectory_id;
    LOG(INFO) << "Test test: " << trajectory_id;

    cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder =
        mapBuilder.map_builder_->GetTrajectoryBuilder(trajectory_id);

    viam::io::ReadFile read_file;
    std::vector<std::string> file_list =
        read_file.listFilesInDirectory(this->data_dir + "/data");
    std::string initial_file = file_list[0];

    if (this->starting_scan_number < 0 ||
        this->starting_scan_number >= int(file_list.size())) {
        std::cerr << "starting_scan_number is out of bounds: "
                  << this->starting_scan_number << std::endl;
        return;
    }

    std::cout << "Beginning to add data....\n";

    std::ofstream myfile;
    myfile.open("log.txt");

    int end_scan_number = int(file_list.size());
    for (int i = this->starting_scan_number; i < end_scan_number; i++) {
        auto measurement = mapBuilder.GetDataFromFile(this->data_dir + "/data",
                                                      initial_file, i);
        if (measurement.ranges.size() > 0) {
            trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
            int num_nodes = mapBuilder.map_builder_->pose_graph()
                                ->GetTrajectoryNodes()
                                .size();

            auto local_poses = GetLocalPoses(mapBuilder);
            if (local_poses.size() > 0) {
                auto latest_local_pose = local_poses.back();
                cartographer::transform::Rigid3d global_pose =
                    GetGlobalPose(mapBuilder, trajectory_id, latest_local_pose);
                myfile << "global_pose: " << global_pose.DebugString()
                       << std::endl;
            }

            if ((num_nodes >= this->starting_scan_number &&
                 num_nodes < this->starting_scan_number + 3) ||
                num_nodes % this->picture_print_interval == 0) {
                PaintMap(mapBuilder.map_builder_,
                         this->data_dir + "/map/images",
                         std::to_string(num_nodes));
            }
        }
    }

    myfile.close();

    // Save the map in a pbstream file
    const std::string map_file = this->data_dir + "/map/" + "test_rdk.pbstream";
    mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
    mapBuilder.map_builder_->SerializeStateToFile(true, map_file);

    mapBuilder.map_builder_->FinishTrajectory(trajectory_id);
    mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
    PaintMap(mapBuilder.map_builder_, this->data_dir + "/map/images", "0");

    return;
}

}  // namespace viam