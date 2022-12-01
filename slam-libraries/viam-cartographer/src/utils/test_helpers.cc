#include "test_helpers.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

namespace viam {
namespace utils {

// NOTE: When using this function, make sure to remove the temporary directory
// and its contents once done processing the files by calling the function
// removeTmpDirectory(tmpdir).
boost::filesystem::path createTmpDirectoryAndAddFiles(
    std::vector<std::string> data_files, std::vector<std::string> map_files) {
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmpdir = boost::filesystem::temp_directory_path() /
                                     boost::filesystem::unique_path();
    bool ok = boost::filesystem::create_directory(tmpdir);
    if (!ok) {
        throw std::runtime_error("could not create directory: " +
                                 tmpdir.string());
    }

    // Create subdirectories
    boost::filesystem::path tmpdir_config = createSubdirectory(tmpdir, "config");
    boost::filesystem::path tmpdir_data = createSubdirectory(tmpdir, "data");
    boost::filesystem::path tmpdir_map = createSubdirectory(tmpdir, "map");

    // Add data files to "data" subdirectory
    for (std::string file : data_files) {
        boost::filesystem::ofstream ofs(tmpdir_data / file);
    }

    // Add map files to "map" subdirectory
    for (std::string file : map_files) {
        boost::filesystem::ofstream ofs(tmpdir_map / file);
    }

    return tmpdir;
}

boost::filesystem::path createSubdirectory(boost::filesystem::path directory, std::string subdirectory_name) {
        boost::filesystem::path subdirectory = directory / subdirectory_name;
        bool ok = boost::filesystem::create_directory(subdirectory);
        if (!ok) {
            boost::filesystem::remove_all(directory);
            throw std::runtime_error("could not create directory: " +
                                    subdirectory.string());
        }
        return subdirectory;
}

void removeTmpDirectory(boost::filesystem::path tmpdir) {
    boost::filesystem::remove_all(tmpdir);
}

}  // namespace utils
}  // namespace viam