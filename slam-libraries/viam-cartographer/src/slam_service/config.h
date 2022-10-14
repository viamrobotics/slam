// This is an Experimental variation of cartographer. It has not yet been
// integrated into RDK.
#ifndef CONFIG_H_
#define CONFIG_H_

#include <atomic>

namespace viam {

static const int checkForShutdownIntervalMicroseconds = 1e5;
static std::atomic<bool> b_continue_session{true};

namespace slam_service {
namespace config {

// Parses and validates the command line arguments. Sets the log level. Throws
// an exception if the arguments are malformed.
int ParseAndValidateConfigParams(int argc, char** argv);

// Parse a config parameter map for a specific variable name and return the
// value as a string. Returns empty if the variable is not found within the map.
std::string ConfigParamParser(std::string map, std::string varName);

}  // namespace config
}  // namespace slam_service
}  // namespace viam

#endif  // CONFIG_H_
