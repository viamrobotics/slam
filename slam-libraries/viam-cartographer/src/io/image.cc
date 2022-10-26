#include "image.h"
#include "cairo_jpg.h"

#include <memory>

#include "cartographer/io/file_writer.h"
#include "glog/logging.h"
#include <string>
#include <iostream>
#include <ostream>

namespace viam {
namespace io {

std::string Image::WriteJpegMem(int quality) {
    cartographer::io::UniqueCairoSurfacePtr surface = GetCairoSurface();
    CHECK_EQ(cairo_surface_status(surface.get()), CAIRO_STATUS_SUCCESS);

    unsigned char *data = NULL;
    size_t len = 0;
    CHECK_EQ(cairo_image_surface_write_to_jpeg_mem(surface.get(), &data, &len, quality),
     CAIRO_STATUS_SUCCESS);
    
    std::string jpeg_img(data, data + len);
    free(data);
    return jpeg_img;
}

}  // namespace io
}  // namespace cartographer
