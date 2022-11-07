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
    cartographer::transform::Rigid3d global_pose;
    {
        std::lock_guard<std::mutex> lk(viam_response_mutex);
        global_pose = latest_global_pose;
    }
    // Setup mapping of pose message to the response. NOTE not using
    // inFrame->set_reference_frame
    PoseInFrame *inFrame = response->mutable_pose();
    Pose *myPose = inFrame->mutable_pose();

    // set pose for our response
    myPose->set_x(global_pose.translation().x());
    myPose->set_y(global_pose.translation().y());
    myPose->set_z(global_pose.translation().z());

    // TODO DATA-531: Remove extraction and conversion of quaternion from the
    // extra field in the response once the Rust spatial math library is
    // available and the desired math can be implemented on the Cartographer
    // side

    google::protobuf::Struct *q;
    google::protobuf::Struct *extra = response->mutable_extra();
    q = extra->mutable_fields()->operator[]("quat").mutable_struct_value();
    q->mutable_fields()->operator[]("real").set_number_value(
        global_pose.rotation().w());
    q->mutable_fields()->operator[]("imag").set_number_value(
        global_pose.rotation().x());
    q->mutable_fields()->operator[]("jmag").set_number_value(
        global_pose.rotation().y());
    q->mutable_fields()->operator[]("kmag").set_number_value(
        global_pose.rotation().z());

    return grpc::Status::OK;
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
        std::stringbuf buffer;
        bool has_points = ExtractPointCloudToBuffer(buffer);
        if (has_points) {
            common::v1::PointCloudObject *pco = response->mutable_point_cloud();
            pco->set_point_cloud(buffer.str());
        } else {
            LOG(ERROR) << "map pointcloud does not have points yet";
            return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                "map pointcloud does not have points yet");
        }
    } else {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                            "mime_type should be \"image/jpeg\" or "
                            "\"pointcloud/pcd\", got \"" +
                                mime_type + "\"");
    }
    return grpc::Status::OK;
}

void SLAMServiceImpl::DetermineActionMode() {
    // Check if there is an apriori map in the path_to_map directory
    std::vector<std::string> map_filenames =
        viam::io::ListSortedFilesInDirectory(path_to_map);

    // Check if there is a *.pbstream map in the path_to_map directory
    for (auto filename : map_filenames) {
        if (filename.find(".pbstream") != std::string::npos) {
            // There is an apriori map present, so we're running either in
            // updating or localization mode.
            if (map_rate_sec.count() == 0) {
                LOG(INFO) << "Running in localization only mode";
                action_mode = SLAMServiceActionMode::LOCALIZING;
                return;
            }
            LOG(INFO) << "Running in updating mode";
            action_mode = SLAMServiceActionMode::UPDATING;
            return;
        }
    }
    LOG(INFO) << "Running in mapping mode";
    action_mode = SLAMServiceActionMode::MAPPING;
}

SLAMServiceActionMode SLAMServiceImpl::GetActionMode() { return action_mode; }

void SLAMServiceImpl::OverwriteMapBuilderParameters() {
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.OverwriteOptimizeEveryNNodes(optimize_every_n_nodes);
        map_builder.OverwriteNumRangeData(num_range_data);
        map_builder.OverwriteMissingDataRayLength(missing_data_ray_length);
        map_builder.OverwriteMaxRange(max_range);
        map_builder.OverwriteMinRange(min_range);
        if (action_mode == SLAMServiceActionMode::LOCALIZING) {
            map_builder.OverwriteMaxSubmapsToKeep(max_submaps_to_keep);
        }
        if (action_mode == SLAMServiceActionMode::UPDATING) {
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

bool SLAMServiceImpl::ExtractPointCloudToBuffer(std::stringbuf &buffer) {
    std::ostream oss(&buffer);
    bool has_points = false;

    cartographer::mapping::MapById<cartographer::mapping::NodeId,
                                   cartographer::mapping::TrajectoryNode>
        trajectory_nodes;
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        trajectory_nodes =
            map_builder.map_builder_->pose_graph()->GetTrajectoryNodes();
    }

    long number_points = 0;
    for (const auto &&trajectory_node : trajectory_nodes) {
        auto point_cloud = trajectory_node.data.constant_data
                               ->filtered_gravity_aligned_point_cloud;
        number_points += point_cloud.size();
    }

    // Write our PCD file, which is written as a binary.
    oss << "VERSION .7\n"
        << "FIELDS x y z rgb\n"
        << "SIZE 4 4 4 4\n"
        << "TYPE F F F I\n"
        << "COUNT 1 1 1 1\n"
        << "WIDTH " << number_points << "\n"
        << "HEIGHT " << 1 << "\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\n"
        << "POINTS " << number_points << "\n"
        << "DATA binary\n";

    for (const auto &&trajectory_node : trajectory_nodes) {
        cartographer::sensor::PointCloud local_gravity_aligned_point_cloud =
            trajectory_node.data.constant_data
                ->filtered_gravity_aligned_point_cloud;

        // We're only applying the translation of the `global_pose` on the
        // point cloud here, as opposed to using both the translation and
        // rotation of the global pose of the trajectory node. The reason for
        // this seems to be that since the point cloud is already gravity
        // aligned, the rotational part of the transformation relative to the
        // world frame is already taken care of.
        cartographer::sensor::PointCloud global_point_cloud =
            cartographer::sensor::TransformPointCloud(
                local_gravity_aligned_point_cloud,
                cartographer::transform::Rigid3f(
                    trajectory_node.data.global_pose.cast<float>()
                        .translation(),
                    cartographer::transform::Rigid3f::Quaternion::Identity()));

        for (auto &&point : global_point_cloud) {
            int rgb = 0;
            buffer.sputn((const char *)&point.position[1], 4);
            buffer.sputn((const char *)&point.position[2], 4);
            buffer.sputn((const char *)&point.position[0], 4);
            buffer.sputn((const char *)&rgb, 4);
            has_points = true;
        }
    }
    return has_points;
}

void SLAMServiceImpl::ProcessData() {
    // Set up and build the MapBuilder
    SetUpMapBuilder();
    DetermineActionMode();

    if (action_mode == SLAMServiceActionMode::UPDATING ||
        action_mode == SLAMServiceActionMode::LOCALIZING) {
        // Check if there is an apriori map in the path_to_map directory
        std::vector<std::string> map_filenames =
            viam::io::ListSortedFilesInDirectory(path_to_map);

        bool found_map = false;
        std::string latest_map_filename;
        for (size_t i = map_filenames.size() - 1; i == 0; i--) {
            if (map_filenames.at(i).find(".pbstream") != std::string::npos) {
                latest_map_filename = map_filenames.at(i);
                found_map = true;
                break;
            }
        }
        if (!found_map) {
            throw std::runtime_error(
                "cannot find maps but they should be present");
        }
        // TODO: The optimize flag will become a flag that can be set by the
        // user. Will be implemented here:
        // https://viam.atlassian.net/browse/DATA-117
        bool optimize = true;
        // load_frozen_trajectory has to be true for LOCALIZING action mode,
        // and false for UPDATING action mode.
        bool load_frozen_trajectory =
            action_mode == SLAMServiceActionMode::LOCALIZING;
        // Load apriori map
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.LoadMapFromFile(latest_map_filename, load_frozen_trajectory,
                                    optimize);
    }

    RunSLAM();
}

std::string SLAMServiceImpl::GetNextDataFileOffline() {
    if (!b_continue_session) {
        return "";
    }
    if (file_list_offline.size() == 0) {
        file_list_offline = viam::io::ListSortedFilesInDirectory(path_to_data);
    }
    if (file_list_offline.size() == 0) {
        throw std::runtime_error("no data in data directory");
    }
    if (current_file_offline == file_list_offline.size()) {
        // This log line is needed by rdk integration tests.
        LOG(INFO) << "Finished processing offline data";
        return "";
    }
    const auto to_return = file_list_offline[current_file_offline];
    current_file_offline++;
    return to_return;
}

std::string SLAMServiceImpl::GetNextDataFileOnline() {
    while (b_continue_session) {
        const auto file_list_online =
            viam::io::ListSortedFilesInDirectory(path_to_data);
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

void SLAMServiceImpl::RunSLAM() {
    cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder;
    int trajectory_id;
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        // Set TrajectoryBuilder
        trajectory_id = map_builder.SetTrajectoryBuilder(&trajectory_builder,
                                                         {kRangeSensorId});
        LOG(INFO) << "Using trajectory ID: " << trajectory_id;
    }

    LOG(INFO) << "Beginning to add data...";

    bool set_start_time = false;
    auto file = GetNextDataFile();

    // define tmp_global_pose here so it always has the previous pose
    cartographer::transform::Rigid3d tmp_global_pose =
        cartographer::transform::Rigid3d();
    while (file != "") {
        if (file.find(".pcd") == std::string::npos) {
            file = GetNextDataFile();
            continue;
        }
        if (!set_start_time) {
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            map_builder.SetStartTime(file);
            set_start_time = true;
        }

        {
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            auto measurement = map_builder.GetDataFromFile(file);
            if (measurement.ranges.size() > 0) {
                trajectory_builder->AddSensorData(kRangeSensorId.id,
                                                  measurement);
                auto local_poses = map_builder.GetLocalSlamResultPoses();
                if (local_poses.size() > 0) {
                    tmp_global_pose = map_builder.GetGlobalPose(
                        trajectory_id, local_poses.back());
                }
            }
        }
        {
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            latest_global_pose = tmp_global_pose;
        }

        // This log line is needed by rdk integration tests.
        VLOG(1) << "Passed sensor data to SLAM " << file;

        file = GetNextDataFile();
    }

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
