// This is an experimental integration of cartographer into RDK.
#include "slam_service.h"

#include <iostream>
#include <string>

#include "../io/file_handler.h"
#include "../io/image.h"
#include "../mapping/map_builder.h"
#include "../utils/slam_service_helpers.h"
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

    // Set pose for our response
    myPose->set_x(global_pose.translation().x());
    myPose->set_y(global_pose.translation().y());
    myPose->set_z(global_pose.translation().z());

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
        return GetJpegMap(request, response);
    } else if (mime_type == "pointcloud/pcd") {
        return GetPointCloudMap(request, response);
    } else {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                            "mime_type should be \"image/jpeg\" or "
                            "\"pointcloud/pcd\", got \"" +
                                mime_type + "\"");
    }
}

::grpc::Status SLAMServiceImpl::GetJpegMap(const GetMapRequest *request,
                                           GetMapResponse *response) {
    std::string jpeg_map = "";
    bool add_pose_marker = request->include_robot_marker();

    // Paint or grab the latest occupancy map in form of a jpeg string
    try {
        {
            std::shared_lock optimization_lock{optimization_shared_mutex,
                                               std::defer_lock};
            if (optimization_lock.try_lock()) {
                // We are able to lock the optimization_shared_mutex, which
                // means that the optimization is not ongoing and we can grab
                // the newest map
                jpeg_map = GetLatestJpegMapString(add_pose_marker);
            } else {
                // We couldn't lock the mutex which means the optimization
                // process locked it and we need to use the backed up latest map
                std::lock_guard<std::mutex> lk(viam_response_mutex);
                if (add_pose_marker) {
                    jpeg_map = latest_jpeg_map_with_marker;
                } else {
                    jpeg_map = latest_jpeg_map_without_marker;
                }
            }
        }
        if (jpeg_map.empty()) {
            return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                "currently no map exists yet");
        }
    } catch (std::exception &e) {
        std::ostringstream oss;
        oss << "error encoding image " << e.what();
        return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
    }

    // Write the jpeg map string to the response
    try {
        response->set_image(jpeg_map);
        return grpc::Status::OK;
    } catch (std::exception &e) {
        std::ostringstream oss;
        oss << "error writing image to response " << e.what();
        return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
    }
}

::grpc::Status SLAMServiceImpl::GetPointCloudMap(const GetMapRequest *request,
                                                 GetMapResponse *response) {
    bool pointcloud_has_points = false;
    std::string pointcloud_map;
    // Write or grab the latest pointcloud map in form of a string
    try {
        std::shared_lock optimization_lock{optimization_shared_mutex,
                                           std::defer_lock};
        if (optimization_lock.try_lock()) {
            // We are able to lock the optimization_shared_mutex, which means
            // that the optimization is not ongoing and we can grab the newest
            // map
            pointcloud_has_points =
                ExtractPointCloudToString(pointcloud_map);
        } else {
            // We couldn't lock the mutex which means the optimization process
            // locked it and we need to use the backed up latest map
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            pointcloud_has_points = latest_pointcloud_map_has_points;
            pointcloud_map = latest_pointcloud_map;
        }
    } catch (std::exception &e) {
        std::ostringstream oss;
        oss << "error encoding pointcloud " << e.what();
        return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
    }

    // Write the pointcloud map string to the response
    try {
        common::v1::PointCloudObject *pco = response->mutable_point_cloud();
        if (!pointcloud_has_points) {
            LOG(ERROR) << "map pointcloud does not have points yet";
            return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                "map pointcloud does not have points yet");
        }
        pco->set_point_cloud(pointcloud_map);
        return grpc::Status::OK;
    } catch (std::exception &e) {
        std::ostringstream oss;
        oss << "error writing pointcloud to response " << e.what();
        return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
    }
}

void SLAMServiceImpl::BackupLatestMap() {
    std::string jpeg_map_with_marker_tmp = GetLatestJpegMapString(true);
    std::string jpeg_map_without_marker_tmp = GetLatestJpegMapString(false);
    std::string pointcloud_map_tmp;
    bool pointcloud_map_has_points_tmp = ExtractPointCloudToString(pointcloud_map_tmp);

    std::lock_guard<std::mutex> lk(viam_response_mutex);
    latest_jpeg_map_with_marker = std::move(jpeg_map_with_marker_tmp);
    latest_jpeg_map_without_marker = std::move(jpeg_map_without_marker_tmp);
    latest_pointcloud_map_has_points = pointcloud_map_has_points_tmp;
    latest_pointcloud_map = std::move(pointcloud_map_tmp);
}

ActionMode SLAMServiceImpl::GetActionMode() { return action_mode; }

void SLAMServiceImpl::SetActionMode() {
    action_mode = viam::utils::DetermineActionMode(path_to_map, map_rate_sec);
}

void SLAMServiceImpl::OverwriteMapBuilderParameters() {
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.OverwriteOptimizeEveryNNodes(optimize_every_n_nodes);
        map_builder.OverwriteNumRangeData(num_range_data);
        map_builder.OverwriteMissingDataRayLength(missing_data_ray_length);
        map_builder.OverwriteMaxRange(max_range);
        map_builder.OverwriteMinRange(min_range);
        if (action_mode == ActionMode::LOCALIZING) {
            map_builder.OverwriteMaxSubmapsToKeep(max_submaps_to_keep);
        }
        if (action_mode == ActionMode::UPDATING) {
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
    if (action_mode == ActionMode::MAPPING) {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SetUp(configuration_directory,
                          configuration_mapping_basename);
    } else if (action_mode == ActionMode::LOCALIZING) {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SetUp(configuration_directory,
                          configuration_localization_basename);
    } else if (action_mode == ActionMode::UPDATING) {
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

std::string SLAMServiceImpl::GetLatestJpegMapString(bool add_pose_marker) {
    cartographer::mapping::MapById<
        cartographer::mapping::SubmapId,
        cartographer::mapping::PoseGraphInterface::SubmapPose>
        submap_poses;
    std::map<cartographer::mapping::SubmapId,
             cartographer::mapping::proto::SubmapQuery::Response>
        response_protos;

    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        submap_poses =
            map_builder.map_builder_->pose_graph()->GetAllSubmapPoses();

        for (const auto &&submap_id_pose : submap_poses) {
            cartographer::mapping::proto::SubmapQuery::Response
                &response_proto = response_protos[submap_id_pose.id];
            const std::string error = map_builder.map_builder_->SubmapToProto(
                submap_id_pose.id, &response_proto);
            if (error != "") {
                throw std::runtime_error(error);
            }
        }
    }

    std::map<cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
        submap_slices;

    if (submap_poses.size() == 0) {
        return "";
    }

    for (const auto &&submap_id_pose : submap_poses) {
        auto submap_textures =
            absl::make_unique<::cartographer::io::SubmapTextures>();
        submap_textures->version =
            response_protos[submap_id_pose.id].submap_version();
        for (const auto &texture_proto :
             response_protos[submap_id_pose.id].textures()) {
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
    }

    cartographer::io::PaintSubmapSlicesResult painted_slices =
        viam::io::PaintSubmapSlices(submap_slices, kPixelSize);
    if (add_pose_marker) {
        PaintMarker(painted_slices);
    }

    auto image = io::Image(std::move(painted_slices.surface));
    return image.WriteJpegToString(50);
}

void SLAMServiceImpl::PaintMarker(
    cartographer::io::PaintSubmapSlicesResult &painted_slices) {
    cartographer::transform::Rigid3d global_pose;
    {
        std::lock_guard<std::mutex> lk(viam_response_mutex);
        global_pose = latest_global_pose;
    }
    viam::io::DrawPoseOnSurface(&painted_slices, global_pose, kPixelSize);
}

bool SLAMServiceImpl::ExtractPointCloudToString(std::string &pointcloud) {
    cartographer::mapping::MapById<cartographer::mapping::NodeId,
                                   cartographer::mapping::TrajectoryNode>
        trajectory_nodes;
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        trajectory_nodes =
            map_builder.map_builder_->pose_graph()->GetTrajectoryNodes();
    }

    bool pointcloud_has_points = false;
    std::stringbuf pointcloud_buffer;
    std::ostream oss(&pointcloud_buffer);

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
            pointcloud_buffer.sputn((const char *)&point.position[1], 4);
            pointcloud_buffer.sputn((const char *)&point.position[2], 4);
            pointcloud_buffer.sputn((const char *)&point.position[0], 4);
            pointcloud_buffer.sputn((const char *)&rgb, 4);
            pointcloud_has_points = true;
        }
    }
    pointcloud = pointcloud_buffer.str();
    return pointcloud_has_points;
}

void SLAMServiceImpl::RunSLAM() {
    LOG(INFO) << "Setting up cartographer";
    // Set up and build the MapBuilder
    SetUpMapBuilder();
    SetActionMode();

    double data_start_time = 0;
    if (action_mode == ActionMode::UPDATING ||
        action_mode == ActionMode::LOCALIZING) {
        // Check if there is an apriori map in the path_to_map directory
        std::string latest_map_filename =
            viam::utils::GetLatestMapFilename(path_to_map);
        // load_frozen_trajectory has to be true for LOCALIZING action mode,
        // and false for UPDATING action mode.
        bool load_frozen_trajectory = (action_mode == ActionMode::LOCALIZING);
        if (optimize_on_start) {
            BackupLatestMap();
            std::unique_lock optimization_lock{optimization_shared_mutex,
                                               std::defer_lock};
            optimization_lock.lock();
            // Load apriori map
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            map_builder.LoadMapFromFile(
                latest_map_filename, load_frozen_trajectory, optimize_on_start);
        } else {
            // Load apriori map
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            map_builder.LoadMapFromFile(
                latest_map_filename, load_frozen_trajectory, optimize_on_start);
        }
        data_start_time =
            viam::io::ReadTimeFromTimestamp(latest_map_filename.substr(
                latest_map_filename.find(viam::io::filename_prefix) +
                    viam::io::filename_prefix.length(),
                latest_map_filename.find(".pbstream")));
    }

    LOG(INFO) << "Starting to run cartographer";
    ProcessDataAndStartSavingMaps(data_start_time);
    LOG(INFO) << "Done running cartographer";
}

std::string SLAMServiceImpl::GetNextDataFileOffline() {
    if (!b_continue_session) {
        return "";
    }
    if (file_list_offline.size() == 0) {
        file_list_offline = viam::io::ListSortedFilesInDirectory(path_to_data);
    }
    // We're setting the minimum required files to be two for the following
    // reasons:
    // 1. Cartographer needs at least two PCD files to work properly.
    // 2. A .DS_Store file is frequently added to the data directory when
    // a user opens the directory on osx.
    // Expecting a minimum of 3 files solves both problems without having to
    // loop over and count the number of actual data files in the data
    // directory.
    if (file_list_offline.size() <= 2) {
        throw std::runtime_error("not enough data in data directory");
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
    if (offline_flag) {
        return GetNextDataFileOffline();
    }
    return GetNextDataFileOnline();
}

void SLAMServiceImpl::StartSaveMap() {
    if (map_rate_sec == std::chrono::seconds(0)) {
        return;
    }
    thread_save_map_with_timestamp =
        new std::thread([&]() { this->SaveMapWithTimestamp(); });
}

void SLAMServiceImpl::StopSaveMap() {
    if (map_rate_sec == std::chrono::seconds(0)) {
        return;
    }
    thread_save_map_with_timestamp->join();
}

void SLAMServiceImpl::SaveMapWithTimestamp() {
    auto check_for_shutdown_interval_usec =
        std::chrono::microseconds(checkForShutdownIntervalMicroseconds);
    while (b_continue_session) {
        auto start = std::chrono::high_resolution_clock::now();
        // Sleep for map_rate_sec duration, but check frequently for
        // shutdown
        while (b_continue_session) {
            std::chrono::duration<double, std::milli> time_elapsed_msec =
                std::chrono::high_resolution_clock::now() - start;
            if ((time_elapsed_msec >= map_rate_sec) ||
                (offline_flag && finished_processing_offline)) {
                break;
            }
            if (map_rate_sec - time_elapsed_msec >=
                check_for_shutdown_interval_usec) {
                std::this_thread::sleep_for(check_for_shutdown_interval_usec);
            } else {
                std::this_thread::sleep_for(map_rate_sec - time_elapsed_msec);
                break;
            }
        }

        const std::string filename_with_timestamp =
            viam::io::MakeFilenameWithTimestamp(path_to_map);

        if (offline_flag && finished_processing_offline) {
            {
                std::lock_guard<std::mutex> lk(map_builder_mutex);
                map_builder.SaveMapToFile(true, filename_with_timestamp);
            }
            LOG(INFO) << "Finished saving final optimized map";
            return;
        }

        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SaveMapToFile(true, filename_with_timestamp);
    }
}

void SLAMServiceImpl::ProcessDataAndStartSavingMaps(double data_start_time) {
    // Prepare the trajectory builder and grab the active trajectory_id
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

    // Define tmp_global_pose here so it always has the previous pose
    cartographer::transform::Rigid3d tmp_global_pose =
        cartographer::transform::Rigid3d();
    while (file != "") {
        // Ignore files that are not *.pcd files
        if (file.find(".pcd") == std::string::npos) {
            file = GetNextDataFile();
            continue;
        }
        if (!set_start_time) {
            // Go past files that are not supposed to be included in this run
            double file_time = viam::io::ReadTimeFromTimestamp(
                file.substr(file.find(viam::io::filename_prefix) +
                                viam::io::filename_prefix.length(),
                            file.find(".pcd")));
            if (file_time < data_start_time) {
                file = GetNextDataFile();
                continue;
            }

            // Set the start time if it has not yet been set and
            // start saving maps
            {
                std::lock_guard<std::mutex> lk(map_builder_mutex);
                map_builder.SetStartTime(file);
                set_start_time = true;
            }
            LOG(INFO) << "Starting to save maps...";
            StartSaveMap();
        }
        // Add data to the map_builder to add to the map
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
        // Save a copy of the global pose
        {
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            latest_global_pose = tmp_global_pose;
        }

        // This log line is needed by rdk integration tests.
        VLOG(1) << "Passed sensor data to SLAM " << file;

        file = GetNextDataFile();
    }

    if (!set_start_time) {
        throw std::runtime_error("did not find valid data for the given setup");
    }

    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.map_builder_->FinishTrajectory(trajectory_id);
    }
    if (offline_flag) {
        BackupLatestMap();
        {
            std::unique_lock<std::shared_mutex> optimization_lock(
                optimization_shared_mutex, std::defer_lock);
            optimization_lock.lock();

            std::lock_guard<std::mutex> lk(map_builder_mutex);
            LOG(INFO)
                << "Starting to optimize final map. Can take a little while...";
            map_builder.map_builder_->pose_graph()->RunFinalOptimization();

            auto local_poses = map_builder.GetLocalSlamResultPoses();
            if (local_poses.size() > 0) {
                tmp_global_pose = map_builder.GetGlobalPose(trajectory_id,
                                                            local_poses.back());
            }
        }

        {
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            latest_global_pose = tmp_global_pose;
        }
        finished_processing_offline = true;
        LOG(INFO) << "Finished optimizing final map";

        while (viam::b_continue_session) {
            LOG(INFO) << "Standing by to continue serving requests\n";
            std::this_thread::sleep_for(std::chrono::microseconds(
                viam::checkForShutdownIntervalMicroseconds));
        }
    }
    StopSaveMap();
    LOG(INFO) << "Stopped saving maps";
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
