// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#ifndef VIAM_CARTOGRAPHER_MAP_BUILDER_H_
#define VIAM_CARTOGRAPHER_MAP_BUILDER_H_

#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace viam {
namespace mapping {

class MapBuilder {
   public:
    void SetUp(std::string configuration_directory,
               std::string configuration_basename);

    void BuildMapBuilder();

    cartographer::mapping::MapBuilderInterface::LocalSlamResultCallback
    GetLocalSlamResultCallback();

    cartographer::sensor::TimedPointCloudData GetDataFromFile(
        std::string data_directory, std::string initial_filename, int i);

    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
    cartographer::mapping::proto::MapBuilderOptions map_builder_options_;
    cartographer::mapping::proto::TrajectoryBuilderOptions
        trajectory_builder_options_;
    std::vector<::cartographer::transform::Rigid3d> GetLocalSlamResultPoses();

    cartographer::transform::Rigid3d GetGlobalPose(
        int trajectory_id,
        cartographer::transform::Rigid3d& latest_local_pose_);

    void OverwriteOptimizeEveryNNodes(int value);
    void OverwriteNumRangeData(int value);
    void OverwriteMissingDataRayLength(float value);
    void OverwriteMaxRange(float value);
    void OverwriteMinRange(float value);
    void OverwriteMaxSubmapsToKeep(int value);
    void OverwriteFreshSubmapsCount(int value);
    void OverwriteMinCoveredArea(double value);
    void OverwriteMinAddedSubmapsCount(int value);
    void OverwriteOccupiedSpaceWeight(double value);
    void OverwriteTranslationWeight(double value);
    void OverwriteRotationWeight(double value);

   private:
    std::vector<::cartographer::transform::Rigid3d> local_slam_result_poses_;
};

}  // namespace mapping
}  // namespace viam

#endif  // VIAM_CARTOGRAPHER_MAP_BUILDER_H_
