// This is an Experimental variation of cartographer. It has not yet been integrated into RDK.
#ifndef VIAM_CARTOGRAPHER_IO_SUBMAP_PAINTER_H_
#define VIAM_CARTOGRAPHER_IO_SUBMAP_PAINTER_H_

#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/transform/rigid_transform.h"

namespace viam {
namespace io {

cartographer::io::PaintSubmapSlicesResult PaintSubmapSlices(
    const std::map<::cartographer::mapping::SubmapId,
                   cartographer::io::SubmapSlice>& submaps,
    double resolution);

}  // namespace io
}  // namespace viam

#endif  // VIAM_CARTOGRAPHER_IO_SUBMAP_PAINTER_H_
