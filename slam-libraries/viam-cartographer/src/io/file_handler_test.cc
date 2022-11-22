#include "file_handler.h"

#include <boost/test/unit_test.hpp>
#include <exception>

namespace viam {
namespace io {

BOOST_AUTO_TEST_SUITE(file_handler)

BOOST_AUTO_TEST_CASE(MakeFilenameWithTimestamp) {
    std::string path_to_dir = "path_to_dir";
    std::time_t start_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string filename = MakeFilenameWithTimestamp(path_to_dir);
    std::time_t end_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    // Check if the filename beginning is as expected
    std::string path_appendix = "/map_data_";
    std::string filename_start = filename.substr(0, path_to_dir.length() + path_appendix.length());
    BOOST_TEST(filename_start.compare(path_to_dir + path_appendix) == 0);
    // Extract timestamp
    double filename_time = ReadTimeFromFilename(filename.substr(
        filename.find(filename_prefix) + filename_prefix.length(),
        filename.find(".pcd")));
    // Check if timestamp is between start_time and end_time
    BOOST_TEST((double)start_time <= filename_time);
    BOOST_TEST(filename_time >= (double)end_time);
}

BOOST_AUTO_TEST_CASE(ListSortedFilesInDirectory) {
    // Create a temp directory with a few files with timestamps
    std::vector<std::string> files{"rplidar_data_2022-01-01T01:00:00.0000Z.pcd",
                         "rplidar_data_2022-01-01T01:00:00.0001Z.pcd",
                         "rplidar_data_2022-01-01T01:00:00.0002Z.pcd",
                         "rplidar_data_2022-01-01T01:00:00.0003Z.pcd"};
    // Create a unique path in the temp directory and add the files
    fs::path tmpdir = fs::temp_directory_path() / fs::unique_path();
    bool ok = fs::create_directory(tmpdir);
    if (!ok) {
        throw std::runtime_error("could not create directory: " + tmpdir.string())
    }
    for (std::string file : files) {
        fs::ofstream ofs(tmpdir / file);
        ofs.close();
    }
    // List the sorted files in the directory
    std::vector<std::string> listedFiles = ListSortedFilesInDirectory(tmpdir);
    // Check to make sure that the files in the directory are what we added and in the same order
    BOOST_TEST(files.length() == listedFiles.length());
    for (int i = 0; i < files.length(); i++) {
        BOOST_TEST(files.at(i).compare(listedFiles.at(i)));
    }
    // Close the file and remove the temporary directory and its contents
    fs::remove_all(tmpdir);
}

// test RemoveFile
BOOST_AUTO_TEST_CASE(RemoveFile) {
    // Create a temp directory with a few files with timestamps
    std::vector<std::string> files{"rplidar_data_2022-01-01T01:00:00.0000Z.pcd",
                         "rplidar_data_2022-01-01T01:00:00.0001Z.pcd",
                         "rplidar_data_2022-01-01T01:00:00.0002Z.pcd",
                         "rplidar_data_2022-01-01T01:00:00.0003Z.pcd"};
    // Create a unique path in the temp directory and add the files
    fs::path tmpdir = fs::temp_directory_path() / fs::unique_path();
    bool ok = fs::create_directory(tmpdir);
    if (!ok) {
        throw std::runtime_error("could not create directory: " + tmpdir.string())
    }
    for (std::string file : files) {
        fs::ofstream ofs(tmpdir / file);
        ofs.close();
    }
    // Remove a file
    int success = RemoveFile(tmpdir + files.at(1));
    files.erase(files.begin() + 1);
    // List the files in the directory and check if the right number of files and the right files are still in the directory
    std::vector<std::string> listedFiles = ListSortedFilesInDirectory(tmpdir);
    BOOST_TEST(files.length() == listedFiles.length());
    for (int i = 0; i < files.length(); i++) {
        BOOST_TEST(files.at(i).compare(listedFiles.at(i)));
    }
    // Close the file and remove the temporary directory and its contents
    fs::remove_all(tmpdir);
}

BOOST_AUTO_TEST_CASE(TimedPointCloudDataFromPCDBuilder) {
    // Create a mini PCD file and save it in a tmp directory
    std::string filename = "rplidar_data_2022-01-01T01:00:00.0001Z.pcd";
    std::string pcd = "VERSION .7\n" +
		"FIELDS x y z rgb\n" +
		"SIZE 4 4 4 4\n" +
		"TYPE F F F I\n" +
		"COUNT 1 1 1 1\n" +
		"WIDTH 3\n" +
		"HEIGHT 1\n" +
		"VIEWPOINT 0 0 0 1 0 0 0\n" +
		"POINTS 3\n" +
		"DATA ascii\n" +
		"-0.001000 0.002000 0.005000 16711938\n" +
		"0.582000 0.012000 0.000000 16711938\n" +
		"0.007000 0.006000 0.001000 16711938\n"
    // Create a unique path in the temp directory and add the files
    fs::path tmpdir = fs::temp_directory_path() / fs::unique_path();
    bool ok = fs::create_directory(tmpdir);
    if (!ok) {
        throw std::runtime_error("could not create directory: " + tmpdir.string())
    }
    fs::ofstream ofs(tmpdir / filename);
    ofs << pcd;
    ofs.close();
    // Read it in and check if the data in the TimedPointCloudData is equivalent to what we had in the pcd file
    cartographer::sensor::TimedPointCloudData timed_pcd = TimedPointCloudDataFromPCDBuilder(tmpdir + "/" + filename, 0);
    
    BOOST_TEST(timed_pcd.ranges.length() == 3);
    cartographer::sensor::TimedRangefinderPoint point_1 = timed_pcd.ranges.at(0);
    cartographer::sensor::TimedRangefinderPoint point_2 = timed_pcd.ranges.at(1);
    cartographer::sensor::TimedRangefinderPoint point_3 = timed_pcd.ranges.at(2);

    auto tolerance = boost::test_tools::tolerance(0.00001);
    BOOST_TEST(point_1.position.at(0) == -0.001, tolerance)
    BOOST_TEST(point_1.position.at(1) == 0.002, tolerance)
    BOOST_TEST(point_1.position.at(2) == 0.005, tolerance)

    BOOST_TEST(point_2.position.at(0) == 0.582, tolerance)
    BOOST_TEST(point_2.position.at(1) == 0.012, tolerance)
    BOOST_TEST(point_2.position.at(2) == 0.000, tolerance)

    BOOST_TEST(point_3.position.at(0) == 0.007, tolerance)
    BOOST_TEST(point_3.position.at(1) == 0.006, tolerance)
    BOOST_TEST(point_3.position.at(2) == 0.001, tolerance)
    // Close the file and remove the temporary directory and its contents
    fs::remove_all(tmpdir);
}

BOOST_AUTO_TEST_CASE(ReadTimeFromFilename) {
    // Provide a filename with a timestamp
    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), time_format.c_str(),
                  std::gmtime(&t));
    std::string filename = "rplidar_data_" + timestamp + ".pcd";
    // Read the time 
    double filename_time = ReadTimeFromFilename(filename.substr(
        filename.find(filename_prefix) + filename_prefix.length(),
        filename.find(".pcd")));
    auto tolerance = boost::test_tools::tolerance(0.00001);
    // Make sure the time read from the filename equals what we put into the filename
    BOOST_TEST((double)t == filename_time, tolerance)
}

}
}