// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#include "cartographer/mapping/map_builder.h"

#include "../src/io/read_PCD_file.h"
#include "../src/mapping/map_builder.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "glog/logging.h"

namespace viam {
namespace mapping {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};
double kDuration = 4.;  // Seconds.

std::vector<::cartographer::transform::Rigid3d>
MapBuilder::GetLocalSlamResultPoses() {
    return local_slam_result_poses_;
}

void MapBuilder::SetUp(std::string configuration_directory,
                       std::string configuration_basename) {
    auto file_resolver =
        absl::make_unique<cartographer::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
    const std::string lua_code =
        file_resolver->GetFileContentOrDie(configuration_basename);

    auto options =
        cartographer::common::LuaParameterDictionary::NonReferenceCounted(
            lua_code, std::move(file_resolver));

    auto map_builder_parameters = options->GetDictionary("map_builder");
    auto trajectory_builder_parameters =
        options->GetDictionary("trajectory_builder");

    map_builder_options_ = cartographer::mapping::CreateMapBuilderOptions(
        map_builder_parameters.get());
    trajectory_builder_options_ =
        cartographer::mapping::CreateTrajectoryBuilderOptions(
            trajectory_builder_parameters.get());

    return;
}

void MapBuilder::BuildMapBuilder() {
    map_builder_ =
        cartographer::mapping::CreateMapBuilder(map_builder_options_);
}

cartographer::mapping::MapBuilderInterface::LocalSlamResultCallback
MapBuilder::GetLocalSlamResultCallback() {
    return [=](const int trajectory_id, const ::cartographer::common::Time time,
               const ::cartographer::transform::Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<
                   const cartographer::mapping::TrajectoryBuilderInterface::
                       InsertionResult>) {
        local_slam_result_poses_.push_back(local_pose);
    };
}

cartographer::sensor::TimedPointCloudData MapBuilder::GetDataFromFile(
    std::string data_directory, std::string initial_filename, int i) {
    viam::io::ReadFile read_file;
    std::vector<std::string> files;
    cartographer::sensor::TimedPointCloudData point_cloud;

    files = read_file.listFilesInDirectory(data_directory);

    if (files.size() == 0) {
        LOG(INFO) << "No files found in data directory\n";
        return point_cloud;
    }

    point_cloud =
        read_file.timedPointCloudDataFromPCDBuilder(files[i], initial_filename);

    LOG(INFO) << "----------PCD-------";
    LOG(INFO) << "Time: " << point_cloud.time;
    LOG(INFO) << "Range (size): " << point_cloud.ranges.size();
    LOG(INFO) << "Range start (time): " << point_cloud.ranges[0].time;
    LOG(INFO) << "Range end (time): " << (point_cloud.ranges.back()).time;
    LOG(INFO) << "-----------------\n";

    return point_cloud;
}

// TODO: There might still be a lot of room to improve accuracy & speed.
// Might be worth investigating in the future.
cartographer::transform::Rigid3d MapBuilder::GetGlobalPose(
    int trajectory_id,
    cartographer::transform::Rigid3d& latest_local_pose_) {
    auto local_transform =
        this->map_builder_->pose_graph()->GetLocalToGlobalTransform(
            trajectory_id);
    return local_transform * latest_local_pose_;
}

void MapBuilder::OverwriteOptimizeEveryNNodes(int value) {
    auto mutable_pose_graph_options =
        this->map_builder_options_.mutable_pose_graph_options();
    mutable_pose_graph_options->set_optimize_every_n_nodes(
        value);
}

void MapBuilder::OverwriteNumRangeData(int value) {
    auto mutable_trajectory_builder_2d_options =
        this->trajectory_builder_options_
            .mutable_trajectory_builder_2d_options();
    mutable_trajectory_builder_2d_options->mutable_submaps_options()
        ->set_num_range_data(value);
}

void MapBuilder::OverwriteNumRangeData(float value) {
    auto mutable_trajectory_builder_2d_options =
        this->trajectory_builder_options_
            .mutable_trajectory_builder_2d_options();
    mutable_trajectory_builder_2d_options->set_missing_data_ray_length(
        value);
}

void MapBuilder::OverwriteMaxRange(float value) {
    auto mutable_trajectory_builder_2d_options =
        this->trajectory_builder_options_
            .mutable_trajectory_builder_2d_options();
    mutable_trajectory_builder_2d_options->set_max_range(value);
}

void MapBuilder::OverwriteMinRange(float value) {
    auto mutable_trajectory_builder_2d_options =
        this->trajectory_builder_options_
            .mutable_trajectory_builder_2d_options();
    mutable_trajectory_builder_2d_options->set_min_range(value);
}

void MapBuilder::OverwriteMaxSubmapsToKeep(int value) {
    this->trajectory_builder_options_
        .mutable_pure_localization_trimmer()
        ->set_max_submaps_to_keep(value);
}

void MapBuilder::OverwriteFreshSubmapsCount(int value) {
    auto mutable_pose_graph_options =
        this->map_builder_options_.mutable_pose_graph_options();
    auto mutable_overlapping_submaps_trimmer_2d =
        mutable_pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
    mutable_overlapping_submaps_trimmer_2d->set_fresh_submaps_count(
        value);
}

void MapBuilder::OverwriteMinCoveredArea(double value) {
    auto mutable_pose_graph_options =
        this->map_builder_options_.mutable_pose_graph_options();
    auto mutable_overlapping_submaps_trimmer_2d =
        mutable_pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
    mutable_overlapping_submaps_trimmer_2d->set_min_covered_area(
        value);
}

void MapBuilder::OverwriteMinAddedSubmapsCount(int value) {
    auto mutable_pose_graph_options =
        this->map_builder_options_.mutable_pose_graph_options();
    auto mutable_overlapping_submaps_trimmer_2d =
        mutable_pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
    mutable_overlapping_submaps_trimmer_2d->set_min_added_submaps_count(
        value);
}

void MapBuilder::OverwriteOccupiedSpaceWeight(double value) {
    auto mutable_pose_graph_options =
        this->map_builder_options_.mutable_pose_graph_options();
    auto mutable_ceres_scan_matcher_options =
        mutable_pose_graph_options->mutable_constraint_builder_options()
            ->mutable_ceres_scan_matcher_options();

    mutable_ceres_scan_matcher_options->set_occupied_space_weight(
        value);
}

void MapBuilder::OverwriteTranslationWeight(double value) {
    auto mutable_pose_graph_options =
        this->map_builder_options_.mutable_pose_graph_options();
    auto mutable_ceres_scan_matcher_options =
        mutable_pose_graph_options->mutable_constraint_builder_options()
            ->mutable_ceres_scan_matcher_options();

    mutable_ceres_scan_matcher_options->set_translation_weight(
        value);
}

void MapBuilder::OverwriteRotationWeight(double value) {
    auto mutable_pose_graph_options =
        this->map_builder_options_.mutable_pose_graph_options();
    auto mutable_ceres_scan_matcher_options =
        mutable_pose_graph_options->mutable_constraint_builder_options()
            ->mutable_ceres_scan_matcher_options();

    mutable_ceres_scan_matcher_options->set_rotation_weight(value);
}

}  // namespace mapping
}  // namespace viam
