// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#include "utils.h"

namespace viam {
namespace utils {

const std::string MakeFilenameWithTimestamp(std::string path_to_dir) {
    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), time_format.c_str(),
                  std::gmtime(&t));
    return path_to_dir + "/" + "map_data_" + timestamp + ".pbstream";
}

}
}
