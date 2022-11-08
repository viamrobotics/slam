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
static const int filenamePrefixLength = 6;

std::vector<std::string> ListSortedFilesInDirectory(std::string data_directory);

cartographer::sensor::TimedPointCloudData TimedPointCloudDataFromPCDBuilder(
    std::string file_path, double start_time);

int RemoveFile(std::string);

// Converts UTC time string to a double value.
double ReadTimeFromFilename(std::string filename);

}  // namespace io
}  // namespace viam

#endif  // FILE_HANDLER_H_
