#include "slam_service.h"

#include <iostream>
#include <string>

#include "../io/image.h"
#include "../mapping/map_builder.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"

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
    auto mime_type = request->mime_type();
    response->set_mime_type(mime_type);

    if (mime_type == "image/jpeg") {
        // TODO: Check for request->include_robot_marker() and
        // paint map accordingly with or without the marker
        // included. Ticket: https://viam.atlassian.net/browse/DATA-657
        std::string jpeg_img = "";
        try {
            jpeg_img = PaintMap();
            if (jpeg_img == "") {
                return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                    "currently no map exists yet");
            }
        } catch (std::exception &e) {
            std::ostringstream oss;
            oss << "error encoding image " << e.what();
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
        }

        // Write the image to the response.
        try {
            response->set_image(jpeg_img);
        } catch (std::exception &e) {
            std::ostringstream oss;
            oss << "error writing image to response " << e.what();
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
        }
    } else if (mime_type == "pointcloud/pcd") {
        LOG(ERROR) << "GetMap for mime_type \"pointcloud/pcd\" is not yet "
                      "implemented.\n";
        return grpc::Status(
            grpc::StatusCode::UNIMPLEMENTED,
            "GetMap for mime_type \"pointcloud/pcd\" is not yet implemented.");
    } else {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                            "mime_type should be \"image/jpeg\" or "
                            "\"pointcloud/pcd\", got \"" +
                                mime_type + "\"");
    }
    return grpc::Status::OK;
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
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.OverwriteOptimizeEveryNNodes(optimize_every_n_nodes);
        map_builder.OverwriteNumRangeData(num_range_data);
        map_builder.OverwriteMissingDataRayLength(missing_data_ray_length);
        map_builder.OverwriteMaxRange(max_range);
        map_builder.OverwriteMinRange(min_range);
        if (slam_action_mode == SLAMServiceActionMode::LOCALIZING) {
            map_builder.OverwriteMaxSubmapsToKeep(max_submaps_to_keep);
        }
        if (slam_action_mode == SLAMServiceActionMode::UPDATING) {
            map_builder.OverwriteFreshSubmapsCount(fresh_submaps_count);
            map_builder.OverwriteMinCoveredArea(min_covered_area);
            map_builder.OverwriteMinAddedSubmapsCount(min_added_submaps_count);
        }
        map_builder.OverwriteOccupiedSpaceWeight(occupied_space_weight);
        map_builder.OverwriteTranslationWeight(translation_weight);
        map_builder.OverwriteRotationWeight(rotation_weight);
    }
}

void SLAMServiceImpl::SetUpMapBuilder() {
    auto action_mode = GetActionMode();
    if (action_mode == SLAMServiceActionMode::MAPPING) {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SetUp(configuration_directory,
                          configuration_mapping_basename);
    } else if (action_mode == SLAMServiceActionMode::LOCALIZING) {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SetUp(configuration_directory,
                          configuration_localization_basename);
    } else if (action_mode == SLAMServiceActionMode::UPDATING) {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SetUp(configuration_directory,
                          configuration_update_basename);
    } else {
        throw std::runtime_error("invalid action mode");
    }
    OverwriteMapBuilderParameters();
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    map_builder.BuildMapBuilder();
}

std::string SLAMServiceImpl::PaintMap() {
    const double kPixelSize = 0.01;
    cartographer::mapping::MapById<
        cartographer::mapping::SubmapId,
        cartographer::mapping::PoseGraphInterface::SubmapPose>
        submap_poses;
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        submap_poses =
            map_builder.map_builder_->pose_graph()->GetAllSubmapPoses();
    }
    std::map<cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
        submap_slices;

    if (submap_poses.size() == 0) {
        return "";
    }

    for (const auto &submap_id_pose : submap_poses) {
        cartographer::mapping::proto::SubmapQuery::Response response_proto;

        {
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            const std::string error = map_builder.map_builder_->SubmapToProto(
                submap_id_pose.id, &response_proto);
            if (error != "") {
                throw std::runtime_error(error);
            }
        }

        auto submap_textures =
            absl::make_unique<::cartographer::io::SubmapTextures>();
        submap_textures->version = response_proto.submap_version();
        for (const auto &texture_proto : response_proto.textures()) {
            const std::string compressed_cells(texture_proto.cells().begin(),
                                               texture_proto.cells().end());
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
        ::cartographer::io::SubmapSlice &submap_slice =
            submap_slices[submap_id_pose.id];
        const auto fetched_texture = submap_textures->textures.begin();
        submap_slice.pose = submap_id_pose.data.pose;
        submap_slice.width = fetched_texture->width;
        submap_slice.height = fetched_texture->height;
        submap_slice.slice_pose = fetched_texture->slice_pose;
        submap_slice.resolution = fetched_texture->resolution;
        submap_slice.cairo_data.clear();

        submap_slice.surface = ::cartographer::io::DrawTexture(
            fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
            fetched_texture->width, fetched_texture->height,
            &submap_slice.cairo_data);

        if (submap_id_pose.id.submap_index == 0 &&
            submap_id_pose.id.trajectory_id == 0) {
            cartographer::mapping::MapById<
                cartographer::mapping::NodeId,
                cartographer::mapping::TrajectoryNode>
                trajectory_nodes;
            {
                std::lock_guard<std::mutex> lk(map_builder_mutex);
                trajectory_nodes = map_builder.map_builder_->pose_graph()
                                       ->GetTrajectoryNodes();
            }
            submap_slice.surface = viam::io::DrawTrajectoryNodes(
                trajectory_nodes, submap_slice.resolution,
                submap_slice.slice_pose, submap_slice.surface.get());
        }
    }

    cartographer::io::PaintSubmapSlicesResult painted_slices =
        viam::io::PaintSubmapSlices(submap_slices, kPixelSize);
    auto image = viam::io::Image(std::move(painted_slices.surface));
    std::string jpeg_img = image.WriteJpegToString(50);
    return jpeg_img;
}

void SLAMServiceImpl::ProcessData() {
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

std::string SLAMServiceImpl::GetNextDataFileOffline() {
    if (!b_continue_session) {
        return "";
    }
    if (file_list_offline.size() == 0) {
        file_list_offline = viam::io::ListFilesInDirectory(path_to_data);
    }
    if (file_list_offline.size() == 0) {
        throw std::runtime_error("no data in data directory");
    }
    if (current_file_offline == file_list_offline.size()) {
        LOG(INFO) << "Finished processing offline images";
        return "";
    }
    const auto to_return = file_list_offline[current_file_offline];
    current_file_offline++;
    return to_return;
}

std::string SLAMServiceImpl::GetNextDataFileOnline() {
    while(b_continue_session) {
        const auto file_list_online = viam::io::ListFilesInDirectory(path_to_data);
        if (file_list_online.size() > 1) {
            // Get the second-most-recent file, since the most-recent file may
            // still be being written.
            const auto to_return =
                file_list_online[file_list_online.size() - 2];
            if (to_return.compare(current_file_online) != 0) {
                current_file_online = to_return;
                return to_return;
            }
        }
        VLOG(1) << "No new files found";
        std::this_thread::sleep_for(data_rate_ms);
    }
    return "";
}

std::string SLAMServiceImpl::GetNextDataFile() {
    if (offlineFlag) {
        return GetNextDataFileOffline();
    }
    return GetNextDataFileOnline();
}

void SLAMServiceImpl::CreateMap() {
    cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder;
    int trajectory_id;
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        // Build TrajectoryBuilder
        trajectory_id = map_builder.map_builder_->AddTrajectoryBuilder(
            {kRangeSensorId}, map_builder.trajectory_builder_options_,
            map_builder.GetLocalSlamResultCallback());

        LOG(INFO) << "Trajectory ID: " << trajectory_id;

        trajectory_builder =
            map_builder.map_builder_->GetTrajectoryBuilder(trajectory_id);
    }

    LOG(INFO) << "Beginning to add data...";

    std::ofstream myfile;
    myfile.open("log.txt");

    bool set_start_time = false;
    auto file = GetNextDataFile();
    while (file != "") {
        if (!set_start_time) {
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            map_builder.SetStartTime(file);
            set_start_time = true;
        }

        int num_nodes;
        {
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            auto measurement = map_builder.GetDataFromFile(file);
            if (measurement.ranges.size() > 0) {
                trajectory_builder->AddSensorData(kRangeSensorId.id,
                                                  measurement);
                num_nodes = map_builder.map_builder_->pose_graph()
                                ->GetTrajectoryNodes()
                                .size();

                auto local_poses = map_builder.GetLocalSlamResultPoses();
                if (local_poses.size() > 0) {
                    auto latest_local_pose = local_poses.back();
                    cartographer::transform::Rigid3d global_pose =
                        map_builder.GetGlobalPose(trajectory_id,
                                                  latest_local_pose);
                    myfile << "global_pose: " << global_pose.DebugString()
                           << std::endl;
                }
            }
        }

        file = GetNextDataFile();
    }

    myfile.close();

    if (!set_start_time) return;

    // Save the map in a pbstream file
    const std::string map_file = path_to_map + "/map.pbstream";
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.map_builder_->pose_graph()->RunFinalOptimization();
        map_builder.map_builder_->SerializeStateToFile(true, map_file);

        map_builder.map_builder_->FinishTrajectory(trajectory_id);
        map_builder.map_builder_->pose_graph()->RunFinalOptimization();
    }

    LOG(INFO) << "Finished optimizing final map";
    return;
}

int SLAMServiceImpl::GetOptimizeEveryNNodesFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetOptimizeEveryNNodes();
}

int SLAMServiceImpl::GetNumRangeDataFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetNumRangeData();
}

float SLAMServiceImpl::GetMissingDataRayLengthFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMissingDataRayLength();
}

float SLAMServiceImpl::GetMaxRangeFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMaxRange();
}

float SLAMServiceImpl::GetMinRangeFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMinRange();
}

int SLAMServiceImpl::GetMaxSubmapsToKeepFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMaxSubmapsToKeep();
}

int SLAMServiceImpl::GetFreshSubmapsCountFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetFreshSubmapsCount();
}

double SLAMServiceImpl::GetMinCoveredAreaFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMinCoveredArea();
}

int SLAMServiceImpl::GetMinAddedSubmapsCountFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMinAddedSubmapsCount();
}

double SLAMServiceImpl::GetOccupiedSpaceWeightFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetOccupiedSpaceWeight();
}

double SLAMServiceImpl::GetTranslationWeightFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetTranslationWeight();
}

double SLAMServiceImpl::GetRotationWeightFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetRotationWeight();
}

}  // namespace viam
