#include "image.h"
#include "cairo_jpg.h"

#include <memory>

#include "cartographer/io/file_writer.h"
#include "glog/logging.h"

namespace viam {
namespace io {

void Image::WriteJpegMem(std::vector<unsigned char> buffer, int quality) {
    cartographer::io::UniqueCairoSurfacePtr surface = GetCairoSurface();
    CHECK_EQ(cairo_surface_status(surface.get()), CAIRO_STATUS_SUCCESS);
    unsigned char *data = NULL;
    size_t len = 0;
    CHECK_EQ(cairo_image_surface_write_to_jpeg_mem(surface.get(), &data, &len, quality),
     CAIRO_STATUS_SUCCESS);
    for (int i = 0; i < len; i++) {
      buffer.push_back(data[i]);
    }
    free(data);
}

}  // namespace io
}  // namespace cartographer
