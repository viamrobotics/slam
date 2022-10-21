#include "slam_service.h"

#include <boost/test/unit_test.hpp>
#include <exception>

namespace viam {
namespace {

BOOST_AUTO_TEST_SUITE(SLAMService)

// TODO[kat]: Test OverwriteMapBuilderParameters

BOOST_AUTO_TEST_CASE(GetActionMode_mapping) {
    SLAMServiceImpl slamService;
    slamService.map_rate_sec = std::chrono::seconds(60);
    BOOST_TEST((slamService.GetActionMode() == SLAMServiceActionMode::MAPPING));
}

/*
BOOST_AUTO_TEST_CASE(GetActionMode_updating) {
    // TODO: Add a test case GetActionMode for updating.Requires that an apriori
    // map is detected and loaded. Will be implemented in this ticket:
    // https://viam.atlassian.net/browse/DATA-114
    BOOST_TEST((slamService.GetActionMode() == SLAMServiceActionMode::UPDATING));
}
*/

/*
BOOST_AUTO_TEST_CASE(GetActionMode_localizing) {
    // TODO: Once this scope doc is approved:
    // https://docs.google.com/document/d/1RsT-c0QOtkMKa-rwUGY0-emUmKIRSaO3mzT1v6MtFjk/edit#
    SLAMServiceImpl slamService;
    slamService.map_rate_sec = -1; // Would need refactoring
    BOOST_TEST((slamService.GetActionMode() == SLAMServiceActionMode::LOCALIZING));
}
*/


BOOST_AUTO_TEST_SUITE_END()

}  // namespace
}  // namespace viam