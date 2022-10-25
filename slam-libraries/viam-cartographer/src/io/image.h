/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VIAM_IO_IMAGE_H_
#define VIAM_IO_IMAGE_H_

#include <cstdint>
#include <vector>

#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/color.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/image.h"

namespace viam {
namespace io {

class Image : public cartographer::io::Image {
 public:
  using cartographer::io::Image::Image;
  std::string WriteJpegMem(int quality);
};

}  // namespace io
}  // namespace cartographer

#endif  // VIAM_IO_IMAGE_H_
