#include "slam_service.h"

#include <boost/test/unit_test.hpp>
#include <exception>

namespace viam {
namespace {

BOOST_AUTO_TEST_SUITE(SLAMService)

void checkCartoMapBuilderParameters(SLAMServiceImpl& slamService) {
    auto pose_graph_options =
        slamService.map_builder.map_builder_options_.pose_graph_options();
    auto overlapping_submaps_trimmer_2d =
        pose_graph_options.overlapping_submaps_trimmer_2d();
    auto ceres_scan_matcher_options =
        pose_graph_options.constraint_builder_options()
            .ceres_scan_matcher_options();
    auto trajectory_builder_2d_options =
        slamService.map_builder.trajectory_builder_options_
            .trajectory_builder_2d_options();

    auto tolerance = boost::test_tools::tolerance(0.00001);
    BOOST_TEST(pose_graph_options.optimize_every_n_nodes() ==
               slamService.optimize_every_n_nodes);
    BOOST_TEST(
        trajectory_builder_2d_options.submaps_options().num_range_data() ==
        slamService.num_range_data);
    BOOST_TEST(trajectory_builder_2d_options.missing_data_ray_length() ==
                   slamService.missing_data_ray_length,
               tolerance);
    BOOST_TEST(
        trajectory_builder_2d_options.max_range() == slamService.max_range,
        tolerance);
    BOOST_TEST(
        trajectory_builder_2d_options.min_range() == slamService.min_range,
        tolerance);
    if (slamService.GetActionMode() == SLAMServiceActionMode::LOCALIZING) {
        BOOST_TEST(slamService.map_builder.trajectory_builder_options_
                       .pure_localization_trimmer()
                       .max_submaps_to_keep() ==
                   slamService.max_submaps_to_keep);
    } else {
        BOOST_TEST(slamService.map_builder.trajectory_builder_options_
                       .pure_localization_trimmer()
                       .max_submaps_to_keep() == 0);
    }
    if (slamService.GetActionMode() == SLAMServiceActionMode::UPDATING) {
        BOOST_TEST(overlapping_submaps_trimmer_2d.fresh_submaps_count() ==
                   slamService.fresh_submaps_count);
        BOOST_TEST(overlapping_submaps_trimmer_2d.min_covered_area() ==
                       slamService.min_covered_area,
                   tolerance);
        BOOST_TEST(overlapping_submaps_trimmer_2d.min_added_submaps_count() ==
                   slamService.min_added_submaps_count);
    } else {
        BOOST_TEST(overlapping_submaps_trimmer_2d.fresh_submaps_count() == 0);
        BOOST_TEST(overlapping_submaps_trimmer_2d.min_covered_area() == 0,
                   tolerance);
        BOOST_TEST(overlapping_submaps_trimmer_2d.min_added_submaps_count() ==
                   0);
    }
    BOOST_TEST(ceres_scan_matcher_options.occupied_space_weight() ==
                   slamService.occupied_space_weight,
               tolerance);
    BOOST_TEST(ceres_scan_matcher_options.translation_weight() ==
                   slamService.translation_weight,
               tolerance);
    BOOST_TEST(ceres_scan_matcher_options.rotation_weight() ==
                   slamService.rotation_weight,
               tolerance);
}

BOOST_AUTO_TEST_CASE(OverwriteMapBuilderParameters_set_values_mapping) {
    SLAMServiceImpl slamService;
    slamService.map_rate_sec = std::chrono::seconds(60);
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
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);
}

BOOST_AUTO_TEST_CASE(
    OverwriteMapBuilderParameters_check_default_values_mapping) {
    SLAMServiceImpl slamService;
    slamService.map_rate_sec = std::chrono::seconds(60);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);
}

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

BOOST_AUTO_TEST_SUITE_END()

}  // namespace
}  // namespace viam
