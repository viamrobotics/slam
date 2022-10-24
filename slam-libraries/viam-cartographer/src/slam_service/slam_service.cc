#include "slam_service.h"

#include <iostream>
#include <string>

#include "../mapping/map_builder.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"
#include "server_functions.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/id.h"

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

    float max = 0;
    float min = 10000;
    auto mime_type = request->mime_type();
    response->set_mime_type(mime_type);

    PaintMap(this->path_to_map + "/images",
                std::to_string(num_nodes));


    LOG(ERROR) << "GetMap is not yet implemented.\n";
    return grpc::Status(grpc::StatusCode::UNIMPLEMENTED,
                        "GetMap is not yet implemented.");
}

SLAMServiceActionMode SLAMServiceImpl::GetActionMode() {
    // TODO: Add a case for updating. Requires that an apriori
    // map is detected and loaded. Will be implemented in this ticket:
    // https://viam.atlassian.net/browse/DATA-114

    // TODO: Change this depending on the outcome of this scope
    // doc:
    // https://docs.google.com/document/d/1RsT-c0QOtkMKa-rwUGY0-emUmKIRSaO3mzT1v6MtFjk/edit#heading=h.tcicyojyqi6c
    if (map_rate_sec.count() == -1) {
        LOG(INFO) << "Running in localization only mode";
        return SLAMServiceActionMode::LOCALIZING;
    }
    LOG(INFO) << "Running in mapping mode";
    return SLAMServiceActionMode::MAPPING;
}

void SLAMServiceImpl::OverwriteMapBuilderParameters() {
    SLAMServiceActionMode slam_action_mode = GetActionMode();

    {
        std::lock_guard<std::mutex> lk(slam_mutex);
        auto mutable_pose_graph_options =
            map_builder.map_builder_options_.mutable_pose_graph_options();
        mutable_pose_graph_options->set_optimize_every_n_nodes(
            optimize_every_n_nodes);

        auto mutable_trajectory_builder_2d_options =
            map_builder.trajectory_builder_options_
                .mutable_trajectory_builder_2d_options();
        mutable_trajectory_builder_2d_options->mutable_submaps_options()
            ->set_num_range_data(num_range_data);
        mutable_trajectory_builder_2d_options->set_missing_data_ray_length(
            missing_data_ray_length);
        mutable_trajectory_builder_2d_options->set_max_range(max_range);
        mutable_trajectory_builder_2d_options->set_min_range(min_range);
        if (slam_action_mode == SLAMServiceActionMode::LOCALIZING) {
            map_builder.trajectory_builder_options_
                .mutable_pure_localization_trimmer()
                ->set_max_submaps_to_keep(max_submaps_to_keep);
        }

        auto mutable_overlapping_submaps_trimmer_2d =
            mutable_pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
        if (slam_action_mode == SLAMServiceActionMode::UPDATING) {
            mutable_overlapping_submaps_trimmer_2d->set_fresh_submaps_count(
                fresh_submaps_count);
            mutable_overlapping_submaps_trimmer_2d->set_min_covered_area(
                min_covered_area);
            mutable_overlapping_submaps_trimmer_2d->set_min_added_submaps_count(
                min_added_submaps_count);
        }

        auto mutable_ceres_scan_matcher_options =
            mutable_pose_graph_options->mutable_constraint_builder_options()
                ->mutable_ceres_scan_matcher_options();
        mutable_ceres_scan_matcher_options->set_occupied_space_weight(
            occupied_space_weight);
        mutable_ceres_scan_matcher_options->set_translation_weight(
            translation_weight);
        mutable_ceres_scan_matcher_options->set_rotation_weight(rotation_weight);
    }
}

void SLAMServiceImpl::SetUpMapBuilder() {
    auto action_mode = GetActionMode();
    {
        std::lock_guard<std::mutex> lk(slam_mutex);
        if (action_mode == SLAMServiceActionMode::MAPPING) {
            map_builder.SetUp(this->configuration_directory,
                            this->configuration_mapping_basename);
        } else if (action_mode == SLAMServiceActionMode::LOCALIZING) {
            map_builder.SetUp(this->configuration_directory,
                            this->configuration_localization_basename);
        } else if (action_mode == SLAMServiceActionMode::UPDATING) {
            map_builder.SetUp(this->configuration_directory,
                            this->configuration_update_basename);
        } else {
            throw std::runtime_error("invalid action mode");
        }
        OverwriteMapBuilderParameters();
        map_builder.BuildMapBuilder();
    }
}

void SLAMServiceImpl::PaintMap(
    std::string output_directory, std::string appendix) {
    const double kPixelSize = 0.01;
    cartographer::mapping::MapById<cartographer::mapping::SubmapId, cartographer::mapping::SubmapPose> submap_poses;
    {
        std::lock_guard<std::mutex> lk(slam_mutex);
        submap_poses = map_builder.map_builder_->pose_graph()->GetAllSubmapPoses();
    }
    std::map<cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
        submap_slices;

    if (submap_poses.size() > 0) {
        for (const auto& submap_id_pose : submap_poses) {
            cartographer::mapping::proto::SubmapQuery::Response response_proto;

            {
                std::lock_guard<std::mutex> lk(slam_mutex);
                const std::string error =
                    map_builder.map_builder_->SubmapToProto(submap_id_pose.id, &response_proto);
                if (error != "") {
                    throw std::runtime_error(error);
                }
            }

            auto submap_textures =
                absl::make_unique<::cartographer::io::SubmapTextures>();
            submap_textures->version = response_proto.submap_version();
            for (const auto& texture_proto : response_proto.textures()) {
                const std::string compressed_cells(
                    texture_proto.cells().begin(), texture_proto.cells().end());
                submap_textures->textures.emplace_back(
                    ::cartographer::io::SubmapTexture{
                        ::cartographer::io::UnpackTextureData(
                            compressed_cells, texture_proto.width(),
                            texture_proto.height()),
                        texture_proto.width(), texture_proto.height(),
                        texture_proto.resolution(),
                        cartographer::transform::ToRigid3(
                            texture_proto.slice_pose())});
            }

            // Prepares SubmapSlice
            ::cartographer::io::SubmapSlice& submap_slice =
                submap_slices[submap_id_pose.id];
            const auto fetched_texture = submap_textures->textures.begin();
            submap_slice.pose = submap_id_pose.data.pose;
            submap_slice.width = fetched_texture->width;
            submap_slice.height = fetched_texture->height;
            submap_slice.slice_pose = fetched_texture->slice_pose;
            submap_slice.resolution = fetched_texture->resolution;
            submap_slice.cairo_data.clear();

            submap_slice.surface = ::cartographer::io::DrawTexture(
                fetched_texture->pixels.intensity,
                fetched_texture->pixels.alpha, fetched_texture->width,
                fetched_texture->height, &submap_slice.cairo_data);

            if (submap_id_pose.id.submap_index == 0 &&
                submap_id_pose.id.trajectory_id == 0) {
                {
                    std::lock_guard<std::mutex> lk(slam_mutex);
                    const auto trajectory_nodes =
                        map_builder.map_builder_->pose_graph()->GetTrajectoryNodes();
                }
                submap_slice.surface = viam::io::DrawTrajectoryNodes(
                    trajectory_nodes, submap_slice.resolution,
                    submap_slice.slice_pose, submap_slice.surface.get());
            }
        }

        cartographer::io::PaintSubmapSlicesResult painted_slices =
            viam::io::PaintSubmapSlices(submap_slices, kPixelSize);
        auto image = cartographer::io::Image(std::move(painted_slices.surface));
        auto file = cartographer::io::StreamFileWriter(
            output_directory + "/map_" + appendix + ".jpg");
        image.WritePng(&file);
    }
}

void SLAMServiceImpl::ProcessDataOffline() {
    // Set up and build the MapBuilder
    SetUpMapBuilder();

    // TODO: Check whether or not we're running localization and/or
    // updating a map. Create a function to run those modes.
    // Implemented with these tickets:
    // https://viam.atlassian.net/browse/DATA-114
    // https://viam.atlassian.net/browse/DATA-631

    // TODO: Call localization only mode function, if localization only
    // action mode

    // TODO: Call updating map function, if apriori map was loaded

    // Mapping a new map:
    CreateMap();
}

void SLAMServiceImpl::CreateMap() {
    {
        std::lock_guard<std::mutex> lk(slam_mutex);
        // Build TrajectoryBuilder
        int trajectory_id = map_builder.map_builder_->AddTrajectoryBuilder(
            {kRangeSensorId}, map_builder.trajectory_builder_options_,
            map_builder.GetLocalSlamResultCallback());

    LOG(INFO) << "Trajectory ID: " << trajectory_id;
    LOG(INFO) << "Test test: " << trajectory_id;

    cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder =
        map_builder.map_builder_->GetTrajectoryBuilder(trajectory_id);
    }

    viam::io::ReadFile read_file;
    std::vector<std::string> file_list =
        read_file.listFilesInDirectory(this->path_to_data);
    std::string initial_file = file_list[0];

    if (this->starting_scan_number < 0 ||
        this->starting_scan_number >= int(file_list.size())) {
        throw std::runtime_error("starting_scan_number is out of bounds: " +
                                 std::to_string(this->starting_scan_number));
    }

    std::cout << "Beginning to add data....\n";

    std::ofstream myfile;
    myfile.open("log.txt");

    int end_scan_number = int(file_list.size());
    for (int i = this->starting_scan_number; i < end_scan_number; i++) {
        if (!b_continue_session) return;
        {
            std::lock_guard<std::mutex> lk(slam_mutex);
            auto measurement =
                map_builder.GetDataFromFile(this->path_to_data, initial_file, i);
            if (measurement.ranges.size() > 0) {
                trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
                int num_nodes = map_builder.map_builder_->pose_graph()
                                    ->GetTrajectoryNodes()
                                    .size();

                auto local_poses = GetLocalPoses(map_builder);
                if (local_poses.size() > 0) {
                    auto latest_local_pose = local_poses.back();
                    cartographer::transform::Rigid3d global_pose =
                        GetGlobalPose(map_builder, trajectory_id, latest_local_pose);
                    myfile << "global_pose: " << global_pose.DebugString()
                        << std::endl;
                }
            }

        }
        if ((num_nodes >= this->starting_scan_number &&
             num_nodes < this->starting_scan_number + 3) ||
            num_nodes % this->picture_print_interval == 0) {
            PaintMap(this->path_to_map + "/images",
                     std::to_string(num_nodes));
        }
    }

    myfile.close();

    // Save the map in a pbstream file
    const std::string map_file = this->path_to_map + "/map.pbstream";
    {
        std::lock_guard<std::mutex> lk(slam_mutex);
        map_builder.map_builder_->pose_graph()->RunFinalOptimization();
        map_builder.map_builder_->SerializeStateToFile(true, map_file);

        map_builder.map_builder_->FinishTrajectory(trajectory_id);
        map_builder.map_builder_->pose_graph()->RunFinalOptimization();
    }
    PaintMap(this->path_to_map + "/images", "0");

    LOG(INFO) << "Finished processing offline images";

    return;
}

}  // namespace viam
