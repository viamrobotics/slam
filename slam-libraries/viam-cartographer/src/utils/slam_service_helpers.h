// This is an experimental integration of cartographer into RDK.
#ifndef SLAM_SERVICE_HELPER_FUNCTIONS_H_
#define SLAM_SERVICE_HELPER_FUNCTIONS_H_

#include <chrono>
#include <iostream>
#include <string>

namespace viam {

enum class ActionMode { MAPPING, LOCALIZING, UPDATING };
std::ostream& operator<<(std::ostream& os, const ActionMode& action_mode);

namespace utils {

// DetermineActionMode determines the action mode the slam service runs in,
// which is either mapping, updating, or localizing.
ActionMode DetermineActionMode(std::string path_to_map,
                               std::chrono::seconds map_rate_sec);

// GetLatestMapFilename gets the latest map filename that is
// located in path_to_map.
std::string GetLatestMapFilename(std::string path_to_map);

}  // namespace utils
}  // namespace viam

#endif  // SLAM_SERVICE_HELPER_FUNCTIONS_H_
