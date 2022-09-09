#define BOOST_TEST_MODULE orb_grpc_server tests
#include <orbslam_server_v1.h>

#include <boost/test/included/unit_test.hpp>

namespace viam {
namespace {

BOOST_AUTO_TEST_CASE(example_test) {
    char* argv[] = {"./bin/orb_grpc_server", "-data_dir=path_to_data"};
    BOOST_TEST(utils::argParser(2, argv, "-data_dir=") == "path_to_data");
}

}  // namespace
}  // namespace viam
