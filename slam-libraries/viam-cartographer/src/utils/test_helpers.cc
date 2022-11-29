#include "test_helpers.h"
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

namespace viam {
namespace utils {

// NOTE: When using this function, make sure to remove the temporary directory
// and its contents once done processing the files by calling the function
// removeTmpDirectory(tmpdir).
boost::filesystem::path createTmpDirectoryAndAddFiles(std::vector<std::string> data_files,
                                                std::vector<std::string> map_files) {
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmpdir = boost::filesystem::temp_directory_path() /
                                     boost::filesystem::unique_path();
    bool ok = boost::filesystem::create_directory(tmpdir);
    if (!ok) {
        throw std::runtime_error("could not create directory: " +
                                 tmpdir.string());
    }
    // Create the "config" subdirectory
    boost::filesystem::path tmpdir_config = tmpdir / "config";
    ok = boost::filesystem::create_directory(tmpdir_config);
    if (!ok) {
        boost::filesystem::remove_all(tmpdir);
        throw std::runtime_error("could not create directory: " +
                                 tmpdir_config.string());
    }

    // Create the "data" subdirectory
    boost::filesystem::path tmpdir_data = tmpdir / "data";
    ok = boost::filesystem::create_directory(tmpdir_data);
    if (!ok) {
        boost::filesystem::remove_all(tmpdir);
        throw std::runtime_error("could not create directory: " +
                                 tmpdir_data.string());
    }

    // Create the "map" subdirectory
    boost::filesystem::path tmpdir_map = tmpdir / "map";
    ok = boost::filesystem::create_directory(tmpdir_map);
    if (!ok) {
        boost::filesystem::remove_all(tmpdir);
        throw std::runtime_error("could not create directory: " +
                                 tmpdir_map.string());
    }

    // Add data files to "data" subdirectory
    for (std::string file : data_files) {
        boost::filesystem::ofstream ofs(tmpdir_data / file);
        ofs.close();
    }

    // Add map files to "map" subdirectory
    for (std::string file : map_files) {
        boost::filesystem::ofstream ofs(tmpdir_map / file);
        ofs.close();
    }

    return tmpdir;
}

void removeTmpDirectory(boost::filesystem::path tmpdir) {
    boost::filesystem::remove_all(tmpdir);
}

}  // namespace utils
}  // namespace viam