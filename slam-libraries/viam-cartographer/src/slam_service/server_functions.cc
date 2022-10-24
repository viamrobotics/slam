// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#include "server_functions.h"

#include "../io/draw_trajectories.h"
#include "../io/read_PCD_file.h"
#include "../io/submap_painter.h"
#include "../mapping/map_builder.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/map_builder.h"

namespace viam {

std::vector<cartographer::transform::Rigid3d> GetLocalPoses(
    viam::mapping::MapBuilder& mapBuilder) {
    return mapBuilder.GetLocalSlamResultPoses();
}

// TODO: There might still be a lot of room to improve accuracy & speed.
// Might be worth investigating in the future.
cartographer::transform::Rigid3d GetGlobalPose(
    viam::mapping::MapBuilder& mapBuilder, int trajectory_id,
    cartographer::transform::Rigid3d& latest_local_pose_) {
    auto local_transform =
        mapBuilder.map_builder_->pose_graph()->GetLocalToGlobalTransform(
            trajectory_id);
    return local_transform * latest_local_pose_;
}


}  // namespace viam
