#ifndef TEST_HELPERS_H_
#define TEST_HELPERS_H_

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

namespace viam {
namespace utils {

// NOTE: When using this function, make sure to remove the temporary directory
// and its contents once done processing the files by calling the function
// removeTmpDirectory(tmpdir).
boost::filesystem::path createTmpDirectoryAndAddFiles(
    std::vector<std::string> data_files, std::vector<std::string> map_files);

// Remove the temporary directory tmpdir and its contents
void removeTmpDirectory(boost::filesystem::path tmpdir);

}  // namespace utils
}  // namespace viam

#endif  // TEST_HELPERS_H_