#include <System.h>
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_context.h>

#include <atomic>

#include "common/v1/common.grpc.pb.h"
#include "common/v1/common.pb.h"
#include "service/slam/v1/slam.grpc.pb.h"
#include "service/slam/v1/slam.pb.h"

using grpc::ServerContext;
using viam::service::slam::v1::GetMapRequest;
using viam::service::slam::v1::GetMapResponse;
using viam::service::slam::v1::GetPositionRequest;
using viam::service::slam::v1::GetPositionResponse;
using viam::service::slam::v1::SLAMService;

namespace viam {

static const int filenamePrefixLength = 6;
static const int checkForShutdownIntervalMicroseconds = 1e5;
extern std::atomic<bool> b_continue_session;

class SLAMServiceImpl final : public SLAMService::Service {
   public:
    ::grpc::Status GetPosition(ServerContext *context,
                               const GetPositionRequest *request,
                               GetPositionResponse *response) override;

    ::grpc::Status GetMap(ServerContext *context, const GetMapRequest *request,
                          GetMapResponse *response) override;

    void ProcessDataOnline(ORB_SLAM3::System *SLAM);

    void ProcessDataOffline(ORB_SLAM3::System *SLAM);

    // Creates a simple map containing a 2x4x8 rectangular prism with the robot
    // in the center, for testing GetMap and GetPosition.
    void ProcessDataForTesting(ORB_SLAM3::System *SLAM);

    void StartSaveAtlasAsOsa(ORB_SLAM3::System *SLAM);

    void StopSaveAtlasAsOsa();

    string path_to_data;
    string path_to_map;
    string path_to_sequence;
    string path_to_vocab;
    string path_to_settings;
    string slam_mode;
    string slam_port;
    string camera_name;
    chrono::milliseconds frame_delay_msec;
    chrono::seconds map_rate_sec;
    double yamlTime;
    std::atomic<bool> offlineFlag{false};
    bool local_viewer_flag = false;

   private:
    void SaveAtlasAsOsaWithTimestamp(ORB_SLAM3::System *SLAM);

    std::atomic<bool> finished_processing_offline{false};
    std::thread *thread_save_atlas_as_osa_with_timestamp;

    std::mutex slam_mutex;
    std::mutex map_save_mutex;
    Sophus::SE3f poseGrpc;
    std::vector<ORB_SLAM3::MapPoint *> currMapPoints;
};

namespace utils {

enum class FileParserMethod { Recent, Closest };

// find a specific input argument from rdk and write the value to a string.
// Returns empty if the argument is not found.
string ArgParser(const vector<string> &args, const string varName);

// parse a config map for a specific variable name and return the value as a
// string. Returns empty if the variable is not found within the map.
string ConfigMapParser(string map, string varName);

// Parses and validates the command line arguments. Sets the log level. Throws
// an exception if the arguments are malformed.
void ParseAndValidateArguments(const vector<string> &args,
                               SLAMServiceImpl &slamService);

// Converts UTC time string to a double value.
double ReadTimeFromFilename(const string filename);

std::vector<std::string> ListFilesInDirectoryForCamera(
    const std::string data_directory, const std::string extension,
    const std::string camera_name);

// LoadRGB loads in rgb images to be used by ORBSLAM, and
// returns whether the image was loaded successfully
bool LoadRGB(std::string path_to_data, std::string filename, cv::Mat &imRGB);

// LoadRGBD loads in a rgbd pair of images to be used by ORBSLAM, and
// returns whether the current pair is okay
bool LoadRGBD(std::string path_to_data, std::string filename, cv::Mat &imRGB,
              cv::Mat &imDepth);

// Find the next frame based off the current interest given a directory of
// data and time to search from
int FindFrameIndex(const std::vector<std::string> &filesRGB,
                   std::string slam_mode, std::string path_to_data,
                   FileParserMethod interest, double configTime,
                   double *timeInterest);

// Make a filename to a specific location for a sensor with a timestamp
// currently does not support millisecond resolution
string MakeFilenameWithTimestamp(string path_to_dir,string camera_name);

}  // namespace utils
}  // namespace viam
