// This is an Experimental variation of cartographer. It has not yet been integrated into RDK.
#include "server_functions.h"

#include "../io/draw_trajectories.h"
#include "../io/read_PCD_file.h"
#include "../io/submap_painter.h"
#include "../mapping/map_builder.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/map_builder.h"

// ======= NOTE ====== NOTE ====== NOTE ====== NOTE ====== NOTE ====== NOTE
// [by Kat] This file is a WIP - A
// place where I'm experimenting with extracting & rearranging functions Be
// gentle & forgiving on this file, plz :) Will ask for a thorough review of the
// entire file once it is ready - and once approved - will remove this note.
// =========================================================================
// END OF NOTE

// Functions that need to be implemented:
// GetPosition(Pose)
// GetMap (either image or pointcloud, depending on MIME type) --> create two
// functions, and have if/else in the actual server function?

// GetPosition(Pose)
/*
GetPosition (Pose)
    Purpose: This would return the relative position of the robot with respect
to the "origin" of the map, which is the starting point from where the map was
initially created (at least in the case of cartographer) Variables: name = name
of service Return: Pose of the robot from reference frame of map Note: the
latency of getPose should be no greater than n (probably 30ms)
*/

namespace viam {
namespace slam_service {

std::vector<cartographer::transform::Rigid3d> GetLocalPoses(
    viam::mapping::MapBuilder& mapBuilder) {
    return mapBuilder.GetLocalSlamResultPoses();
}

// TODO[kat]: There might still be a lot of room to improve accuracy & speed.
// Might be worth investigating in the future.
cartographer::transform::Rigid3d GetGlobalPose(
    viam::mapping::MapBuilder& mapBuilder, int trajectory_id,
    cartographer::transform::Rigid3d& latest_local_pose_) {
    auto local_transform =
        mapBuilder.map_builder_->pose_graph()->GetLocalToGlobalTransform(
            trajectory_id);
    return local_transform * latest_local_pose_;
}

void PaintMap(
    std::unique_ptr<cartographer::mapping::MapBuilderInterface>& map_builder_,
    std::string output_directory, std::string appendix) {
    const double kPixelSize = 0.01;
    auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
    std::map<cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
        submap_slices;

    if (submap_poses.size() > 0) {
        for (const auto& submap_id_pose : submap_poses) {
            cartographer::mapping::proto::SubmapQuery::Response response_proto;
            const std::string error =
                map_builder_->SubmapToProto(submap_id_pose.id, &response_proto);

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
                const auto trajectory_nodes =
                    map_builder_->pose_graph()->GetTrajectoryNodes();
                submap_slice.surface = viam::io::DrawTrajectoryNodes(
                    trajectory_nodes, submap_slice.resolution,
                    submap_slice.slice_pose, submap_slice.surface.get());
            }
        }

        cartographer::io::PaintSubmapSlicesResult painted_slices =
            viam::io::PaintSubmapSlices(submap_slices, kPixelSize);
        auto image = cartographer::io::Image(std::move(painted_slices.surface));
        auto file = cartographer::io::StreamFileWriter(
            output_directory + "/map_" + appendix + ".png");
        image.WritePng(&file);
    }
}

// GetMap
// -- GetImage
// -- GetPointcloud

/*
GetMap
    Purpose: Allow user to get the custom visualization of the current map
produced by the SLAM library Parameters: Name = name of service mime_type =
requested MIME type of response image/jpeg image/pcd (optional) camera_position
= common.v1.pose (optional) include_robot_marker = include robot position on map
    Optional parameter, defaults to false
    Return:
    map = image or point cloud of specified map (based on mime type)
    Note: use pcd data type in common.proto
    Mime_type = actual mime_type of response


Marker: Red dot overlaid on map
*/

}  // namespace slam_service
}  // namespace viam
