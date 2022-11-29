#include "slam_service.h"
#include "../utils/test_helpers.h"

#include <boost/test/unit_test.hpp>
#include <exception>

namespace viam {
namespace {

BOOST_AUTO_TEST_SUITE(SLAMService)

void checkCartoMapBuilderParameters(SLAMServiceImpl& slamService) {
    auto tolerance = boost::test_tools::tolerance(0.00001);

    BOOST_TEST(slamService.GetOptimizeEveryNNodesFromMapBuilder() ==
               slamService.optimize_every_n_nodes);
    BOOST_TEST(slamService.GetNumRangeDataFromMapBuilder() ==
               slamService.num_range_data);
    BOOST_TEST(slamService.GetMissingDataRayLengthFromMapBuilder() ==
                   slamService.missing_data_ray_length,
               tolerance);
    BOOST_TEST(slamService.GetMaxRangeFromMapBuilder() == slamService.max_range,
               tolerance);
    BOOST_TEST(slamService.GetMinRangeFromMapBuilder() == slamService.min_range,
               tolerance);
    if (slamService.GetActionMode() == SLAMServiceActionMode::LOCALIZING) {
        BOOST_TEST(slamService.GetMaxSubmapsToKeepFromMapBuilder() ==
                   slamService.max_submaps_to_keep);
    } else {
        BOOST_TEST(slamService.GetMaxSubmapsToKeepFromMapBuilder() == 0);
    }

    if (slamService.GetActionMode() == SLAMServiceActionMode::UPDATING) {
        BOOST_TEST(slamService.GetFreshSubmapsCountFromMapBuilder() ==
                   slamService.fresh_submaps_count);
        BOOST_TEST(slamService.GetMinCoveredAreaFromMapBuilder() ==
                       slamService.min_covered_area,
                   tolerance);
        BOOST_TEST(slamService.GetMinAddedSubmapsCountFromMapBuilder() ==
                   slamService.min_added_submaps_count);
    } else {
        BOOST_TEST(slamService.GetFreshSubmapsCountFromMapBuilder() == 0);
        BOOST_TEST(slamService.GetMinCoveredAreaFromMapBuilder() == 0,
                   tolerance);
        BOOST_TEST(slamService.GetMinAddedSubmapsCountFromMapBuilder() == 0);
    }
    BOOST_TEST(slamService.GetOccupiedSpaceWeightFromMapBuilder() ==
                   slamService.occupied_space_weight,
               tolerance);
    BOOST_TEST(slamService.GetTranslationWeightFromMapBuilder() ==
                   slamService.translation_weight,
               tolerance);
    BOOST_TEST(slamService.GetRotationWeightFromMapBuilder() ==
                   slamService.rotation_weight,
               tolerance);
}

BOOST_AUTO_TEST_CASE(OverwriteMapBuilderParameters_set_values_mapping) {
    // Mapping is the default action_mode when slamService is created
    SLAMServiceImpl slamService;
    slamService.optimize_every_n_nodes = 9999;
    slamService.num_range_data = 9998;
    slamService.missing_data_ray_length = 9997.77;
    slamService.max_range = 9996.66;
    slamService.min_range = 9995.55;
    slamService.max_submaps_to_keep = 9994;
    slamService.fresh_submaps_count = 9993;
    slamService.min_covered_area = 9992.22;
    slamService.min_added_submaps_count = 9991;
    slamService.occupied_space_weight = 9990.09;
    slamService.translation_weight = 9989.89;
    slamService.rotation_weight = 9988.88;

    BOOST_TEST(slamService.GetActionMode() ==
                   SLAMServiceActionMode::MAPPING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);
}

BOOST_AUTO_TEST_CASE(
    OverwriteMapBuilderParameters_check_default_values_mapping) {
    // Mapping is the default action_mode when slamService is created
    SLAMServiceImpl slamService;

    BOOST_TEST(slamService.GetActionMode() ==
                   SLAMServiceActionMode::MAPPING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);
}

BOOST_AUTO_TEST_CASE(OverwriteMapBuilderParameters_set_values_updating) {
    SLAMServiceImpl slamService;
    slamService.optimize_every_n_nodes = 9999;
    slamService.num_range_data = 9998;
    slamService.missing_data_ray_length = 9997.77;
    slamService.max_range = 9996.66;
    slamService.min_range = 9995.55;
    slamService.max_submaps_to_keep = 9994;
    slamService.fresh_submaps_count = 9993;
    slamService.min_covered_area = 9992.22;
    slamService.min_added_submaps_count = 9991;
    slamService.occupied_space_weight = 9990.09;
    slamService.translation_weight = 9989.89;
    slamService.rotation_weight = 9988.88;

    // Set the action mode to updating by providing an apriori map
    // and by setting map_rate_sec != 0
    slamService.map_rate_sec = std::chrono::seconds(60);

    // Create a temp directory with a few files with timestamps
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{"map_data_2022-02-11T01:44:53.1903Z.pbstream"};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmpdir = utils::createTmpDirectoryAndAddFiles(data_files, map_files);
    slamService.path_to_map = tmpdir.string() + "/map";
    slamService.DetermineActionMode();

    BOOST_TEST(slamService.GetActionMode() ==
                   SLAMServiceActionMode::UPDATING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmpdir);
}

BOOST_AUTO_TEST_CASE(
    OverwriteMapBuilderParameters_check_default_values_updating) {
    SLAMServiceImpl slamService;
    // Set the action mode to updating by providing an apriori map
    // and by setting map_rate_sec != 0
    slamService.map_rate_sec = std::chrono::seconds(60);

    // Create a temp directory with a few files with timestamps
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{"map_data_2022-02-11T01:44:53.1903Z.pbstream"};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmpdir = utils::createTmpDirectoryAndAddFiles(data_files, map_files);
    slamService.path_to_map = tmpdir.string() + "/map";
    slamService.DetermineActionMode();

    BOOST_TEST(slamService.GetActionMode() ==
                   SLAMServiceActionMode::UPDATING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmpdir);
}

BOOST_AUTO_TEST_CASE(OverwriteMapBuilderParameters_set_values_localizing) {
    SLAMServiceImpl slamService;
    slamService.optimize_every_n_nodes = 9999;
    slamService.num_range_data = 9998;
    slamService.missing_data_ray_length = 9997.77;
    slamService.max_range = 9996.66;
    slamService.min_range = 9995.55;
    slamService.max_submaps_to_keep = 9994;
    slamService.fresh_submaps_count = 9993;
    slamService.min_covered_area = 9992.22;
    slamService.min_added_submaps_count = 9991;
    slamService.occupied_space_weight = 9990.09;
    slamService.translation_weight = 9989.89;
    slamService.rotation_weight = 9988.88;

    // Set the action mode to localizing by providing an apriori map
    // and by setting map_rate_sec == 0
    slamService.map_rate_sec = std::chrono::seconds(0);

    // Create a temp directory with a few files with timestamps
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{"map_data_2022-02-11T01:44:53.1903Z.pbstream"};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmpdir = utils::createTmpDirectoryAndAddFiles(data_files, map_files);
    slamService.path_to_map = tmpdir.string() + "/map";
    slamService.DetermineActionMode();

    BOOST_TEST(slamService.GetActionMode() ==
                   SLAMServiceActionMode::LOCALIZING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmpdir);
}

BOOST_AUTO_TEST_CASE(
    OverwriteMapBuilderParameters_check_default_values_localizing) {
    SLAMServiceImpl slamService;

    // Set the action mode to localizing by providing an apriori map
    // and by setting map_rate_sec == 0
    slamService.map_rate_sec = std::chrono::seconds(0);

    // Create a temp directory with a few files with timestamps
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{"map_data_2022-02-11T01:44:53.1903Z.pbstream"};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmpdir = utils::createTmpDirectoryAndAddFiles(data_files, map_files);
    slamService.path_to_map = tmpdir.string() + "/map";
    slamService.DetermineActionMode();

    BOOST_TEST(slamService.GetActionMode() ==
                   SLAMServiceActionMode::LOCALIZING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmpdir);
}

// TODO: Fix this test
BOOST_AUTO_TEST_CASE(GetActionMode_mapping) {
    SLAMServiceImpl slamService;
    slamService.map_rate_sec = std::chrono::seconds(60);
    BOOST_TEST(slamService.GetActionMode() == SLAMServiceActionMode::MAPPING);
}

/*
BOOST_AUTO_TEST_CASE(GetActionMode_updating) {
    // TODO: Add a test case GetActionMode for updating.Requires that an apriori
    // map is detected and loaded. Will be implemented in this ticket:
    // https://viam.atlassian.net/browse/DATA-114
    BOOST_TEST((slamService.GetActionMode() ==
SLAMServiceActionMode::UPDATING));
}
*/

/*
BOOST_AUTO_TEST_CASE(GetActionMode_localizing) {
    // TODO: Once this scope doc is approved:
    //
https://docs.google.com/document/d/1RsT-c0QOtkMKa-rwUGY0-emUmKIRSaO3mzT1v6MtFjk/edit#
    SLAMServiceImpl slamService;
    slamService.map_rate_sec = -1; // Would need refactoring
    BOOST_TEST((slamService.GetActionMode() ==
SLAMServiceActionMode::LOCALIZING));
}
*/


// ProcessDataAndStartSavingMaps

// GetNextDataFile

// GetNextDataFileOffline

// GetNextDataFileOnline

// RunSLAM

// DetermineActionMode
// How to create different action modes:
// Create a slamService object --> Mapping: Default.
// Provide a path (path_to_map) that contains a map (.pbstream) --> Updating or Localizing
// --> Set map_rate_sec == 0 --> Localizing
// --> Set map_rate_sec != 0 --> Updating
// Test for the error: Set map_rate_sec == 0 but don't provide a map



// GetActionMode

// SetUpMapBuilder

// OverwriteMapBuilderParameters with different values for action mode

// PaintMap

// ExtractPointCloudToBuffer

BOOST_AUTO_TEST_SUITE_END()

}  // namespace
}  // namespace viam
