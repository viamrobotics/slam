// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#ifndef VIAM_READ_FROM_FILE_H_
#define VIAM_READ_FROM_FILE_H_

#include <inttypes.h>

#include <chrono>
#include <ostream>
#include <ratio>
#include <string>

#include "cartographer/sensor/timed_point_cloud_data.h"

namespace viam {
static const int filenamePrefixLength = 6;
namespace io {

class ReadFile {
   public:
    std::string time_format = "%Y-%m-%dT%H:%M:%SZ";
    std::vector<std::string> listFilesInDirectory(std::string data_directory);
    cartographer::sensor::TimedPointCloudData timedPointCloudDataFromPCDBuilder(
        std::string file_path, std::string initial_filename);
    int removeFile(std::string);

    // Converts UTC time string to a double value.
    double ReadFile::ReadTimeFromFilename(std::string filename);
};

}  // namespace io
}  // namespace viam

#endif  // VIAM_READ_FROM_FILE_H_
