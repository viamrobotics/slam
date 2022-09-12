#define BOOST_TEST_MODULE orb_grpc_server tests
#include "orbslam_server_v1.h"

#include <boost/test/included/unit_test.hpp>

namespace viam {
namespace {

void checkParseAndValidateArgumentsException(const vector<string>& args,
                                             const string& message) {
    SLAMServiceImpl slamService;
    BOOST_CHECK_EXCEPTION(utils::parseAndValidateArguments(args, slamService),
                          runtime_error, [&message](const runtime_error& ex) {
                              BOOST_CHECK_EQUAL(ex.what(), message);
                              return true;
                          });
}

BOOST_AUTO_TEST_CASE(parseAndValidateArguments_no_args) {
    const vector<string> args;
    const string message =
        "No args found. Expected: \n"
        "./bin/orb_grpc_server "
        "-data_dir=path_to_data "
        "-config_param={mode=slam_mode,} "
        "-port=grpc_port "
        "-sensors=sensor_name "
        "-data_rate_ms=frame_delay "
        "-map_rate_sec=map_rate_sec";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(parseAndValidateArguments_no_data_dir) {
    const vector<string> args{"-config_param={mode=slam_mode}",
                              "-port=20000",
                              "-sensors=color",
                              "-data_rate_ms=200",
                              "-map_rate_sec=60",
                              "-unknown=unknown"};
    const string message = "No data directory given";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(parseAndValidateArguments_no_slam_mode) {
    const vector<string> args{"-data_dir=/path/to", "-config_param={}",
                              "-port=20000",        "-sensors=color",
                              "-data_rate_ms=200",  "-map_rate_sec=60"};
    const string message = "No SLAM mode given";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(parseAndValidateArguments_no_slam_port) {
    const vector<string> args{
        "-data_dir=/path/to", "-config_param={mode=slam_mode}",
        "-sensors=color",     "-data_rate_ms=200",
        "-map_rate_sec=60",   "-unknown=unknown"};
    const string message = "No gRPC port given";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(parseAndValidateArguments_no_camera_data_rate) {
    const vector<string> args{
        "-data_dir=/path/to", "-config_param={mode=slam_mode}",
        "-port=20000",        "-sensors=color",
        "-map_rate_sec=60",   "-unknown=unknown"};
    const string message = "No camera data rate specified";
    checkParseAndValidateArgumentsException(args, message);
}

BOOST_AUTO_TEST_CASE(parseAndValidateArguments_valid_config) {
    const vector<string> args{
        "-data_dir=/path/to", "-config_param={mode=slam_mode}",
        "-port=20000",        "-sensors=color",
        "-data_rate_ms=200",  "-map_rate_sec=60"};
    SLAMServiceImpl slamService;
    utils::parseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.path_to_vocab == "/path/to/config/ORBvoc.txt");
    BOOST_TEST(slamService.path_to_settings == "/path/to/config");
    BOOST_TEST(slamService.path_to_data == "/path/to/data");
    BOOST_TEST(slamService.path_to_map == "/path/to/map");
    BOOST_TEST(slamService.slam_mode == "slam_mode");
    BOOST_TEST(slamService.slam_port == "20000");
    BOOST_TEST(slamService.frame_delay_msec.count() ==
               chrono::milliseconds(200).count());
    BOOST_TEST(slamService.map_rate_sec.count() == chrono::seconds(60).count());
    BOOST_TEST(slamService.camera_name == "color");
    BOOST_TEST(slamService.offlineFlag == false);
}

BOOST_AUTO_TEST_CASE(
    parseAndValidateArguments_valid_config_capitalized_slam_mode) {
    const vector<string> args{
        "-data_dir=/path/to", "-config_param={mode=SLAM_MODE}",
        "-port=20000",        "-sensors=color",
        "-data_rate_ms=200",  "-map_rate_sec=60"};
    SLAMServiceImpl slamService;
    utils::parseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.slam_mode == "slam_mode");
}

BOOST_AUTO_TEST_CASE(parseAndValidateArguments_valid_config_no_map_rate_sec) {
    const vector<string> args{
        "-data_dir=/path/to", "-config_param={mode=slam_mode}",
        "-port=20000",        "-sensors=color",
        "-data_rate_ms=200",  "-map_rate_sec="};
    SLAMServiceImpl slamService;
    utils::parseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.map_rate_sec.count() == chrono::seconds(0).count());
}

BOOST_AUTO_TEST_CASE(parseAndValidateArguments_valid_config_no_camera) {
    const vector<string> args{
        "-data_dir=/path/to", "-config_param={mode=slam_mode}",
        "-port=20000",        "-sensors=",
        "-data_rate_ms=200",  "-map_rate_sec=60"};
    SLAMServiceImpl slamService;
    utils::parseAndValidateArguments(args, slamService);
    BOOST_TEST(slamService.camera_name == "");
    BOOST_TEST(slamService.offlineFlag == true);
}

BOOST_AUTO_TEST_CASE(readTimeFromFilename) {
    const string filename1 = "2022-01-01T01_00_00.0000";
    const string filename2 = "2022-01-01T01_00_00.0001";
    const string filename3 = "2022-01-01T01_00_01.0000";
    const auto time1 = utils::readTimeFromFilename(filename1);
    const auto time2 = utils::readTimeFromFilename(filename2);
    const auto time3 = utils::readTimeFromFilename(filename3);
    BOOST_TEST(time1 < time2);
    BOOST_TEST(time2 < time3);
}

BOOST_AUTO_TEST_CASE(parseDataDir_Closest_no_files) {
    const string configTimeString = "2022-01-01T01_00_00.0000";
    const auto configTime = utils::readTimeFromFilename(configTimeString);
    vector<string> files;
    double timeInterest;
    BOOST_TEST(utils::parseDataDir(files, utils::FileParserMethod::Closest,
                                   configTime, &timeInterest) == -1);
}

BOOST_AUTO_TEST_CASE(parseDataDir_Closest_ignore_last) {
    const string configTimeString = "2022-01-01T01_00_00.0001";
    const auto configTime = utils::readTimeFromFilename(configTimeString);
    vector<string> files{"color_data_2022-01-01T01_00_00.0000.png",
                         "color_data_2022-01-01T01_00_00.0001.png",
                         "color_data_2022-01-01T01_00_00.0002.png"};
    double timeInterest;
    BOOST_TEST(utils::parseDataDir(files, utils::FileParserMethod::Closest,
                                   configTime, &timeInterest) == -1);
}

BOOST_AUTO_TEST_CASE(parseDataDir_Closest_found_time) {
    const string configTimeString = "2022-01-01T01_00_00.0000";
    const auto configTime = utils::readTimeFromFilename(configTimeString);
    vector<string> files{"color_data_2022-01-01T01_00_00.0000.png",
                         "color_data_2022-01-01T01_00_00.0001.png",
                         "color_data_2022-01-01T01_00_00.0002.png",
                         "color_data_2022-01-01T01_00_00.0003.png"};
    double timeInterest;
    BOOST_TEST(utils::parseDataDir(files, utils::FileParserMethod::Closest,
                                   configTime, &timeInterest) == 1);
    BOOST_TEST(timeInterest ==
               utils::readTimeFromFilename("2022-01-01T01_00_00.0001"));
}

BOOST_AUTO_TEST_CASE(parseDataDir_Recent_no_files) {
    const string configTimeString = "2022-01-01T01_00_00.0000";
    const auto configTime = utils::readTimeFromFilename(configTimeString);
    vector<string> files;
    double timeInterest;
    BOOST_TEST(utils::parseDataDir(files, utils::FileParserMethod::Recent,
                                   configTime, &timeInterest) == -1);
}

BOOST_AUTO_TEST_CASE(parseDataDir_Recent_ignore_last) {
    const string configTimeString = "2022-01-01T01_00_00.0001";
    const auto configTime = utils::readTimeFromFilename(configTimeString);
    vector<string> files{"color_data_2022-01-01T01_00_00.0000.png",
                         "color_data_2022-01-01T01_00_00.0001.png",
                         "color_data_2022-01-01T01_00_00.0002.png"};
    double timeInterest;
    BOOST_TEST(utils::parseDataDir(files, utils::FileParserMethod::Recent,
                                   configTime, &timeInterest) == -1);
}

BOOST_AUTO_TEST_CASE(parseDataDir_Recent_found_time) {
    const string configTimeString = "2022-01-01T01_00_00.0000";
    const auto configTime = utils::readTimeFromFilename(configTimeString);
    vector<string> files{"color_data_2022-01-01T01_00_00.0000.png",
                         "color_data_2022-01-01T01_00_00.0001.png",
                         "color_data_2022-01-01T01_00_00.0002.png",
                         "color_data_2022-01-01T01_00_00.0003.png"};
    double timeInterest;
    BOOST_TEST(utils::parseDataDir(files, utils::FileParserMethod::Recent,
                                   configTime, &timeInterest) == 2);
    BOOST_TEST(timeInterest ==
               utils::readTimeFromFilename("2022-01-01T01_00_00.0002"));
}

}  // namespace
}  // namespace viam
