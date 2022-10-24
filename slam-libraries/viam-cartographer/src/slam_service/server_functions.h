// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#ifndef SERVER_FUNCTIONS_H_
#define SERVER_FUNCTIONS_H_

#include "../mapping/map_builder.h"
#include "cartographer/mapping/map_builder.h"

namespace viam {

std::vector<cartographer::transform::Rigid3d> GetLocalPoses(
    viam::mapping::MapBuilder& mapBuilder);

cartographer::transform::Rigid3d GetGlobalPose(
    viam::mapping::MapBuilder& mapBuilder, int trajectory_id,
    cartographer::transform::Rigid3d& latest_local_pose_);

}  // namespace viam

#endif  // SERVER_FUNCTIONS_H_
