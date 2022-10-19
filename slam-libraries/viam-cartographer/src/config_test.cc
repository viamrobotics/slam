#define BOOST_TEST_MODULE config tests
#include "slam_service/config.h"

#include <boost/test/included/unit_test.hpp>
#include <exception>

namespace viam {
namespace config {
namespace {

void checkParseAndValidateConfigParamsException(int argc, char** argv,
                                                const std::string& message) {
    SLAMServiceImpl slamService;
    BOOST_CHECK_EXCEPTION(ParseAndValidateConfigParams(argc, argv, slamService),
                          std::runtime_error,
                          [&message](const std::runtime_error& ex) {
                              BOOST_CHECK_EQUAL(ex.what(), message);
                              return true;
                          });
}

char** toCharArrayArray(std::vector<std::string>& args) {
    char** argv = new char*[args.size()];
    for (auto i = 0; i < args.size(); i++) {
        argv[i] = &args[i][0];
    }
    return argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_no_config_param) {
    ResetFlagsForTesting();
    std::vector<std::string> args{"carto_grpc_server", "-data_dir=/path/to",
                                  "-port=localhost:0", "-sensors=lidar",
                                  "-data_rate_ms=200", "-map_rate_sec=60"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    const std::string message = "-config_param is missing";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_no_data_dir) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server", "-config_param={mode=2d}", "-port=localhost:0",
        "-sensors=lidar",    "-data_rate_ms=200",       "-map_rate_sec=60"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    const std::string message = "-data_dir is missing";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_no_port) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server", "-config_param={mode=2d}", "-data_dir=/path/to",
        "-sensors=lidar",    "-data_rate_ms=200",       "-map_rate_sec=60"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    const std::string message = "-port is missing";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_no_slam_mode) {
    ResetFlagsForTesting();
    std::vector<std::string> args{"carto_grpc_server",  "-config_param={}",
                                  "-data_dir=/path/to", "-port=localhost:0",
                                  "-sensors=lidar",     "-data_rate_ms=200",
                                  "-map_rate_sec=60"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    const std::string message = "slam mode is missing";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_invalid_slam_mode) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server", "-config_param={mode=bad}", "-data_dir=/path/to",
        "-port=localhost:0", "-sensors=lidar",           "-data_rate_ms=200",
        "-map_rate_sec=60"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    const std::string message = "Invalid slam_mode=bad";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_valid_config) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server", "-config_param={mode=2d}", "-data_dir=/path/to",
        "-port=localhost:0", "-sensors=lidar",          "-data_rate_ms=200",
        "-map_rate_sec=60"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    config::ParseAndValidateConfigParams(argc, argv, slamService);
    BOOST_TEST(slamService.path_to_data == "/path/to/data");
    BOOST_TEST(slamService.path_to_map == "/path/to/map");
    BOOST_TEST(slamService.config_params == "{mode=2d}");
    BOOST_TEST(slamService.slam_mode == "2d");
    BOOST_TEST(slamService.port == "localhost:0");
    BOOST_TEST(slamService.data_rate_ms.count() ==
               std::chrono::milliseconds(200).count());
    BOOST_TEST(slamService.map_rate_sec.count() ==
               std::chrono::seconds(60).count());
    BOOST_TEST(slamService.camera_name == "lidar");
    BOOST_TEST(slamService.offlineFlag == false);
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_valid_config_capitalized_slam_mode) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server", "-config_param={mode=2D}", "-data_dir=/path/to",
        "-port=localhost:0", "-sensors=lidar",          "-data_rate_ms=200",
        "-map_rate_sec=60"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, slamService);
    BOOST_TEST(slamService.slam_mode == "2d");
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_valid_config_no_map_rate_sec) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server", "-config_param={mode=2d}", "-data_dir=/path/to",
        "-port=localhost:0", "-sensors=lidar",          "-data_rate_ms=200"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, slamService);
    BOOST_TEST(slamService.map_rate_sec.count() ==
               std::chrono::seconds(60).count());
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_valid_config_no_data_rate_ms) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server", "-config_param={mode=2d}", "-data_dir=/path/to",
        "-port=localhost:0", "-sensors=lidar",          "-map_rate_sec=60"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, slamService);
    BOOST_TEST(slamService.data_rate_ms.count() ==
               std::chrono::milliseconds(200).count());
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_valid_config_no_camera) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",  "-config_param={mode=2d}",
        "-data_dir=/path/to", "-port=localhost:0",
        "-sensors=",          "-data_rate_ms=200",
        "-map_rate_sec=60"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, slamService);
    BOOST_TEST(slamService.camera_name == "");
    BOOST_TEST(slamService.offlineFlag == true);
    delete argv;
}

}  // namespace
}  // namespace config
}  // namespace viam
