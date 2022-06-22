#ifndef SERVER_FUNCTIONS_H_
#define SERVER_FUNCTIONS_H_

#include "../mapping/map_builder.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/map_builder.h"

// ======= NOTE ====== NOTE ====== NOTE ====== NOTE ====== NOTE ====== NOTE
// [by Kat] This file is a WIP - A
// place where I'm experimenting with extracting & rearranging functions Be
// gentle & forgiving on this file, plz :) Will ask for a thorough review of the
// entire file once it is ready - and once approved - will remove this note.
// ========================================================================
// END OF NOTE

namespace viam {
namespace slam_service {

cartographer::transform::Rigid3d GetGlobalPose(
    viam::mapping::MapBuilder &mapBuilder, int trajectory_id);

void PaintMap(
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> &map_builder_,
    std::string output_directory, std::string appendix);
}  // namespace slam_service
}  // namespace viam

#endif  // SERVER_FUNCTIONS_H_
