// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#ifndef FILE_HANDLER_H_
#define FILE_HANDLER_H_

#include <inttypes.h>

#include <chrono>
#include <ostream>
#include <ratio>
#include <string>

#include "cartographer/sensor/timed_point_cloud_data.h"

namespace viam {
namespace io {
static const std::string filenamePrefix = "_data_";
static const std::string time_format = "%Y-%m-%dT%H:%M:%S.0000Z";

// MakeFilenameWithTimestamp creates a filename for a provided sensor with a
// timestamp. The filename includes the path to the file. Does not support
// millisecond resolution.
const std::string MakeFilenameWithTimestamp(std::string path_to_dir);

// ListSortedFilesInDirectory returns a list of the files in the directory
// sorted by name.
std::vector<std::string> ListSortedFilesInDirectory(std::string data_directory);

// TimedPointCloudDataFromPCDBuilder creates a TimedPointCloudData object
// from a PCD file.
cartographer::sensor::TimedPointCloudData TimedPointCloudDataFromPCDBuilder(
    std::string file_path, double start_time);

// RemoveFile removes the file at the provided path.
int RemoveFile(std::string);

// Converts UTC time string to a double value.
double ReadTimeFromFilename(std::string filename);

}  // namespace io
}  // namespace viam

#endif  // FILE_HANDLER_H_
