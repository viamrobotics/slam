#include "slam_service.h"

#include <iostream>
#include <string>

#include "../mapping/map_builder.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"
#include "server_functions.h"

namespace viam {

std::atomic<bool> b_continue_session{true};

::grpc::Status SLAMServiceImpl::GetPosition(ServerContext *context,
                                            const GetPositionRequest *request,
                                            GetPositionResponse *response) {
    LOG(ERROR) << "GetPosition is not yet implemented.\n";
    return grpc::Status(grpc::StatusCode::UNIMPLEMENTED,
                        "GetPosition is not yet implemented.");
}

::grpc::Status SLAMServiceImpl::GetMap(ServerContext *context,
                                       const GetMapRequest *request,
                                       GetMapResponse *response) {
    LOG(ERROR) << "GetMap is not yet implemented.\n";
    return grpc::Status(grpc::StatusCode::UNIMPLEMENTED,
                        "GetMap is not yet implemented.");
}


SLAMServiceActionMode SLAMServiceImpl::ActionMode() {
    // TODO: Add a case for updating. Requires that an apriori
    // map is detected and loaded. Will be implemented in this ticket:
    // https://viam.atlassian.net/browse/DATA-114
    if (map_rate_sec.count() == -1) {
        LOG(INFO) << "Running in localization only mode";
        return SLAMServiceActionMode::LOCALIZING;
    }
    LOG(INFO) << "Running in mapping mode";
    return SLAMServiceActionMode::MAPPING;
}

// TODO[kat]: Write tests for this
void SLAMServiceImpl::OverwriteMapBuilderParameters() {
    std::cout << "--- Previous values --- " << std::endl;
    std::cout << "optimize_every_n_nodes: "
              << mapBuilder.map_builder_options_.pose_graph_options().optimize_every_n_nodes() << std::endl;
    std::cout << "num_range_data: " << mapBuilder.trajectory_builder_options_.trajectory_builder_2d_options().submaps_options().num_range_data() << std::endl;
    std::cout << "missing_data_ray_length: " << mapBuilder.trajectory_builder_options_.trajectory_builder_2d_options().missing_data_ray_length() << std::endl;
    std::cout << "max_range: " << mapBuilder.trajectory_builder_options_.trajectory_builder_2d_options().max_range() << std::endl;
    std::cout << "min_range: " << mapBuilder.trajectory_builder_options_.trajectory_builder_2d_options().min_range() << std::endl;
    std::cout << "max_submaps_to_keep: " << mapBuilder.trajectory_builder_options_.pure_localization_trimmer().max_submaps_to_keep() << std::endl;
    std::cout << "fresh_submaps_count: " << mapBuilder.map_builder_options_.pose_graph_options().overlapping_submaps_trimmer_2d().fresh_submaps_count() << std::endl;
    std::cout << "min_covered_area: " << mapBuilder.map_builder_options_.pose_graph_options().overlapping_submaps_trimmer_2d().min_covered_area() << std::endl;
    std::cout << "min_added_submaps_count: " << mapBuilder.map_builder_options_.pose_graph_options().overlapping_submaps_trimmer_2d().min_added_submaps_count() << std::endl;
    std::cout << "occupied_space_weight: " << mapBuilder.map_builder_options_.pose_graph_options().constraint_builder_options().ceres_scan_matcher_options().occupied_space_weight() << std::endl;
    std::cout << "translation_weight: " << mapBuilder.map_builder_options_.pose_graph_options().constraint_builder_options().ceres_scan_matcher_options().translation_weight() << std::endl;
    std::cout << "rotation_weight: " << mapBuilder.map_builder_options_.pose_graph_options().constraint_builder_options().ceres_scan_matcher_options().rotation_weight() << std::endl;
    
    SLAMServiceActionMode slam_action_mode = ActionMode();

    mapBuilder.map_builder_options_.mutable_pose_graph_options()->set_optimize_every_n_nodes(optimize_every_n_nodes);
    mapBuilder.trajectory_builder_options_.mutable_trajectory_builder_2d_options()->mutable_submaps_options()->set_num_range_data(num_range_data);
    mapBuilder.trajectory_builder_options_.mutable_trajectory_builder_2d_options()->set_missing_data_ray_length(missing_data_ray_length);
    mapBuilder.trajectory_builder_options_.mutable_trajectory_builder_2d_options()->set_max_range(max_range);
    mapBuilder.trajectory_builder_options_.mutable_trajectory_builder_2d_options()->set_min_range(min_range);
    if (slam_action_mode == SLAMServiceActionMode::LOCALIZING) {
        mapBuilder.trajectory_builder_options_.mutable_pure_localization_trimmer()->set_max_submaps_to_keep(max_submaps_to_keep);
    }
    if (slam_action_mode == SLAMServiceActionMode::UPDATING) {
    mapBuilder.map_builder_options_.mutable_pose_graph_options()->mutable_overlapping_submaps_trimmer_2d()->set_fresh_submaps_count(fresh_submaps_count);
    mapBuilder.map_builder_options_.mutable_pose_graph_options()->mutable_overlapping_submaps_trimmer_2d()->set_min_covered_area(min_covered_area);
    mapBuilder.map_builder_options_.mutable_pose_graph_options()->mutable_overlapping_submaps_trimmer_2d()->set_min_added_submaps_count(min_added_submaps_count);
    }
    mapBuilder.map_builder_options_.mutable_pose_graph_options()->mutable_constraint_builder_options()->mutable_ceres_scan_matcher_options()->set_occupied_space_weight(occupied_space_weight);
    mapBuilder.map_builder_options_.mutable_pose_graph_options()->mutable_constraint_builder_options()->mutable_ceres_scan_matcher_options()->set_translation_weight(translation_weight);
    mapBuilder.map_builder_options_.mutable_pose_graph_options()->mutable_constraint_builder_options()->mutable_ceres_scan_matcher_options()->set_rotation_weight(rotation_weight);

    std::cout << "--- New values --- " << std::endl;
    std::cout << "optimize_every_n_nodes: "
              << mapBuilder.map_builder_options_.pose_graph_options().optimize_every_n_nodes() << std::endl;
    std::cout << "num_range_data: " << mapBuilder.trajectory_builder_options_.trajectory_builder_2d_options().submaps_options().num_range_data() << std::endl;
    std::cout << "missing_data_ray_length: " << mapBuilder.trajectory_builder_options_.trajectory_builder_2d_options().missing_data_ray_length() << std::endl;
    std::cout << "max_range: " << mapBuilder.trajectory_builder_options_.trajectory_builder_2d_options().max_range() << std::endl;
    std::cout << "min_range: " << mapBuilder.trajectory_builder_options_.trajectory_builder_2d_options().min_range() << std::endl;
    std::cout << "max_submaps_to_keep: " << mapBuilder.trajectory_builder_options_.pure_localization_trimmer().max_submaps_to_keep() << std::endl;
    std::cout << "fresh_submaps_count: " << mapBuilder.map_builder_options_.pose_graph_options().overlapping_submaps_trimmer_2d().fresh_submaps_count() << std::endl;
    std::cout << "min_covered_area: " << mapBuilder.map_builder_options_.pose_graph_options().overlapping_submaps_trimmer_2d().min_covered_area() << std::endl;
    std::cout << "min_added_submaps_count: " << mapBuilder.map_builder_options_.pose_graph_options().overlapping_submaps_trimmer_2d().min_added_submaps_count() << std::endl;
    std::cout << "occupied_space_weight: " << mapBuilder.map_builder_options_.pose_graph_options().constraint_builder_options().ceres_scan_matcher_options().occupied_space_weight() << std::endl;
    std::cout << "translation_weight: " << mapBuilder.map_builder_options_.pose_graph_options().constraint_builder_options().ceres_scan_matcher_options().translation_weight() << std::endl;
    std::cout << "rotation_weight: " << mapBuilder.map_builder_options_.pose_graph_options().constraint_builder_options().ceres_scan_matcher_options().rotation_weight() << std::endl;

}

void SLAMServiceImpl::CreateMap() {
    // Add configs
    mapBuilder.SetUp(this->configuration_directory,
                     this->configuration_mapping_basename);

    OverwriteMapBuilderParameters();

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
        read_file.listFilesInDirectory(this->path_to_data);
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
        if (!b_continue_session) return;

        auto measurement =
            mapBuilder.GetDataFromFile(this->path_to_data, initial_file, i);
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
                PaintMap(mapBuilder.map_builder_, this->path_to_map + "/images",
                         std::to_string(num_nodes));
            }
        }
    }

    myfile.close();

    // Save the map in a pbstream file
    const std::string map_file = this->path_to_map + "/map.pbstream";
    mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
    mapBuilder.map_builder_->SerializeStateToFile(true, map_file);

    mapBuilder.map_builder_->FinishTrajectory(trajectory_id);
    mapBuilder.map_builder_->pose_graph()->RunFinalOptimization();
    PaintMap(mapBuilder.map_builder_, this->path_to_map + "/images", "0");

    return;
}

}  // namespace viam
