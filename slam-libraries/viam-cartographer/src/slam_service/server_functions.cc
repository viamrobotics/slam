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

}  // namespace viam
