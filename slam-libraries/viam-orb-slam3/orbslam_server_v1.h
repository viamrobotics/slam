#include <System.h>
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_context.h>

#include <atomic>

#include "proto/api/common/v1/common.grpc.pb.h"
#include "proto/api/common/v1/common.pb.h"
#include "proto/api/service/slam/v1/slam.grpc.pb.h"
#include "proto/api/service/slam/v1/slam.pb.h"

using grpc::ServerContext;
using proto::api::service::slam::v1::GetMapRequest;
using proto::api::service::slam::v1::GetMapResponse;
using proto::api::service::slam::v1::GetPositionRequest;
using proto::api::service::slam::v1::GetPositionResponse;
using proto::api::service::slam::v1::SLAMService;

namespace viam {

static const int filenamePrefixLength = 6;
static const int checkForShutdownIntervalMicroseconds = 1e5;
static std::atomic<bool> b_continue_session{true};

class SLAMServiceImpl final : public SLAMService::Service {
   public:
    ::grpc::Status GetPosition(ServerContext *context,
                               const GetPositionRequest *request,
                               GetPositionResponse *response) override;

    ::grpc::Status GetMap(ServerContext *context, const GetMapRequest *request,
                          GetMapResponse *response) override;

    void process_rgbd_online(ORB_SLAM3::System *SLAM);

    void process_rgbd_offline(ORB_SLAM3::System *SLAM);

    // Creates a simple map containing a 2x4x8 rectangular prism with the robot
    // in the center, for testing GetMap and GetPosition.
    void process_rgbd_for_testing(ORB_SLAM3::System *SLAM);

    void start_save_atlas_as_osa(ORB_SLAM3::System *SLAM);

    void stop_save_atlas_as_osa();

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

   private:
    void save_atlas_as_osa_with_timestamp(ORB_SLAM3::System *SLAM);

    std::atomic<bool> finished_processing_offline{false};
    std::thread *thread_save_atlas_as_osa_with_timestamp;

    std::mutex slam_mutex;
    Sophus::SE3f poseGrpc;
    std::vector<ORB_SLAM3::MapPoint *> currMapPoints;
};

namespace utils {

enum class FileParserMethod { Recent, Closest };

// find a specific input argument from rdk and write the value to a string.
// Returns empty if the argument is not found.
string argParser(const vector<string> &args, const string varName);

// parse a config map for a specific variable name and return the value as a
// string. Returns empty if the variable is not found within the map.
string configMapParser(string map, string varName);

// Parses and validates the command line arguments. Sets the log level. Throws
// an exception if the arguments are malformed.
void parseAndValidateArguments(const vector<string> &args,
                               SLAMServiceImpl &slamService);

// Converts UTC time string to a double value.
double readTimeFromFilename(const string filename);

std::vector<std::string> listFilesInDirectoryForCamera(
    const std::string data_directory, const std::string extension,
    const std::string camera_name);

bool loadRGBD(std::string path_to_data, std::string filename, cv::Mat &imRGB,
              cv::Mat &imDepth);

// Find the next frame based off the current interest given a directory of
// data and time to search from
int parseDataDir(const std::vector<std::string> &files,
                 FileParserMethod interest, double configTime,
                 double *timeInterest);

int parseBothDataDir(std::string path_to_data,
                     const std::vector<std::string> &filesRGB,
                     FileParserMethod interest, double configTime,
                     double *timeInterest);

}  // namespace utils
}  // namespace viam