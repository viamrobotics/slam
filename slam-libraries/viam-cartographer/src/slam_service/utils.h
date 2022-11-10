// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#ifndef UTILS_H_
#define UTILS_H_

#include <string>


namespace viam {
namespace utils {

static const std::string time_format = "%Y-%m-%dT%H:%M:%S.0000Z";

// Create a filename for a provided sensor with a timestamp. The filename
// includes the path to the file. Does not support millisecond resolution.
const std::string MakeFilenameWithTimestamp(std::string path_to_dir);

}
}

#endif  // UTILS_H_
