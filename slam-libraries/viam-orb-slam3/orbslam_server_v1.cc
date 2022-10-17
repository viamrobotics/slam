// This is an Experimental integration of orbslam into RDK.
#include "orbslam_server_v1.h"

#include <algorithm>
#include <cfenv>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>

#pragma STDC FENV_ACCESS ON

using namespace boost::filesystem;
using google::protobuf::Struct;
using viam::common::v1::PointCloudObject;
using viam::common::v1::Pose;
using viam::common::v1::PoseInFrame;

#define IMAGE_SIZE 300
#define MAX_COLOR_VALUE 255
const std::string strRGB = "/rgb";
const std::string strDepth = "/depth";

namespace viam {

std::atomic<bool> b_continue_session{true};

::grpc::Status SLAMServiceImpl::GetPosition(ServerContext *context,
                                            const GetPositionRequest *request,
                                            GetPositionResponse *response) {
    Sophus::SE3f currPose;
    // Copy pose to new location
    {
        std::lock_guard<std::mutex> lk(slam_mutex);
        currPose = poseGrpc;
    }

    // Setup mapping of pose message to the response. NOTE not using
    // inFrame->set_reference_frame yet
    PoseInFrame *inFrame = response->mutable_pose();
    Pose *myPose = inFrame->mutable_pose();
    const auto actualPose = currPose.params();

    // set pose for our response
    myPose->set_x(actualPose[4]);
    myPose->set_y(actualPose[5]);
    myPose->set_z(actualPose[6]);

    // TODO DATA-531: Remove extraction and conversion of quaternion from the
    // extra field in the response once the Rust spatial math library is
    // available and the desired math can be implemented on the orbSLAM side

    BOOST_LOG_TRIVIAL(debug)
        << "Passing robot position: x= " << actualPose[4]
        << " y= " << actualPose[5] << " z= " << actualPose[6]
        << " Real= " << actualPose[3] << " I_mag= " << actualPose[0]
        << " J_mag= " << actualPose[1] << " K_mag= " << actualPose[2];

    google::protobuf::Struct *q;
    google::protobuf::Struct *extra = response->mutable_extra();
    q = extra->mutable_fields()->operator[]("quat").mutable_struct_value();
    q->mutable_fields()->operator[]("real").set_number_value(actualPose[3]);
    q->mutable_fields()->operator[]("imag").set_number_value(actualPose[0]);
    q->mutable_fields()->operator[]("jmag").set_number_value(actualPose[1]);
    q->mutable_fields()->operator[]("kmag").set_number_value(actualPose[2]);

    return grpc::Status::OK;
}

::grpc::Status SLAMServiceImpl::GetMap(ServerContext *context,
                                       const GetMapRequest *request,
                                       GetMapResponse *response) {
    float max = 0;
    float min = 10000;
    auto mime_type = request->mime_type();
    response->set_mime_type(mime_type);
    std::vector<ORB_SLAM3::MapPoint *> actualMap;
    Sophus::SE3f currPose;
    {
        std::lock_guard<std::mutex> lk(slam_mutex);
        actualMap = currMapPoints;
        currPose = poseGrpc;
    }

    if (actualMap.size() == 0) {
        return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                            "currently no map points exist");
    }

    if (mime_type == "image/jpeg") {
        // Determine the height and width of the image. Height is determined
        // using the z values, since we're projecting onto the XZ plan
        // (since z is currently coming out of the lens).
        auto minX = std::numeric_limits<float>::max();
        auto maxX = std::numeric_limits<float>::lowest();
        auto minZ = std::numeric_limits<float>::max();
        auto maxZ = std::numeric_limits<float>::lowest();

        std::vector<float> valsX;
        std::vector<float> valsZ;

        for (auto &&p : actualMap) {
            const auto v = p->GetWorldPos();

            valsX.push_back(v.x());
            valsZ.push_back(v.z());

            minX = std::min(minX, v.x());
            maxX = std::max(maxX, v.x());
            minZ = std::min(minZ, v.z());
            maxZ = std::max(maxZ, v.z());
        }

        // Determine the max and min values based on distance away from mean
        // using standard deviation. The less extreme value is then chosen
        // to be the max/min vlaues in the image in order display the most
        // useful png
        if (valsX.size() > 1) {
            float sigmaLevel = 7.;

            std::feclearexcept(FE_ALL_EXCEPT);
            cv::Scalar meanX, stdevX, meanZ, stdevZ;
            cv::meanStdDev(valsX, meanX, stdevX);
            cv::meanStdDev(valsX, meanZ, stdevZ);

            float minCalX = meanX[0] - sigmaLevel * stdevX[0];
            float maxCalX = meanX[0] + sigmaLevel * stdevX[0];
            float minCalZ = meanZ[0] - sigmaLevel * stdevZ[0];
            float maxCalZ = meanZ[0] + sigmaLevel * stdevZ[0];

            if (std::fetestexcept(FE_OVERFLOW) ||
                std::fetestexcept(FE_UNDERFLOW)) {
                std::ostringstream oss;
                oss << "cannot calculate mean and standard deviation from "
                       "image due to over/underflow";
                return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
            }

            minX = std::max(minX, minCalX);
            maxX = std::min(maxX, maxCalX);
            minZ = std::max(minZ, minCalZ);
            maxZ = std::min(maxZ, maxCalZ);
        }

        // Add robot marker and ensure it exists in the image, varying
        // max/min values as needed
        if (request->include_robot_marker()) {
            const auto actualPose = currPose.params();
            minX = std::min(minX, actualPose[4]);
            maxX = std::max(maxX, actualPose[4]);
            minZ = std::min(minZ, actualPose[6]);
            maxZ = std::max(maxZ, actualPose[6]);
        }

        std::feclearexcept(FE_ALL_EXCEPT);
        const auto originalWidth = maxX - minX;
        const auto originalHeight = maxZ - minZ;
        // Only check for overflow and underflow. Division by zero and
        // domain error would be unexpected. Inexact results are okay.
        if (std::fetestexcept(FE_OVERFLOW) || std::fetestexcept(FE_UNDERFLOW)) {
            std::ostringstream oss;
            oss << "cannot create image from map with min X: " << minX
                << ", max X: " << maxX << ", min Z: " << minZ
                << ", and max Z: " << maxZ;
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
        }

        if (originalWidth <= 0 || originalHeight <= 0) {
            std::ostringstream oss;
            oss << "cannot create image from map with width: " << originalWidth
                << " and height: " << originalHeight;
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
        }

        // Scale to constant size image.
        std::feclearexcept(FE_ALL_EXCEPT);
        const auto widthScale = (IMAGE_SIZE - 2) / originalWidth;
        const auto heightScale = (IMAGE_SIZE - 2) / originalHeight;
        if (std::fetestexcept(FE_OVERFLOW) || std::fetestexcept(FE_UNDERFLOW)) {
            std::ostringstream oss;
            oss << "cannot create image from map with original width: "
                << originalWidth << ", original height: " << originalHeight
                << ", and image size: " << IMAGE_SIZE << "x" << IMAGE_SIZE;
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
        }

        // Create a new cv::Mat that can hold all of the MapPoints.
        cv::Mat mat(IMAGE_SIZE /* rows */, IMAGE_SIZE /* cols */,
                    CV_8UC3 /* RGB */,
                    cv::Scalar::all(0) /* initialize to black */);

        // Add each point to the cv::Mat. Project onto the XZ plane (since
        // z is currently coming out of the lens).
        BOOST_LOG_TRIVIAL(debug)
            << "Adding " << actualMap.size() << " points to image";
        for (auto &&p : actualMap) {
            const auto v = p->GetWorldPos();

            std::feclearexcept(FE_ALL_EXCEPT);
            const auto j_float = widthScale * (v.x() - minX);
            const auto i_float = heightScale * (v.z() - minZ);
            if (std::fetestexcept(FE_OVERFLOW) ||
                std::fetestexcept(FE_UNDERFLOW)) {
                std::ostringstream oss;
                oss << "cannot scale point with X: " << v.x()
                    << " and Z: " << v.z()
                    << " to include on map with min X: " << minX
                    << ", min Z: " << minZ << ", widthScale: " << widthScale
                    << ", and heightScale: " << heightScale;
                return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
            }

            if (i_float < 0 || i_float >= IMAGE_SIZE || j_float < 0 ||
                j_float >= IMAGE_SIZE) {
                continue;
            }

            const auto j = static_cast<int>(j_float);
            const auto i = static_cast<int>(i_float);
            auto &matPoint = mat.at<cv::Vec3b>(cv::Point(i, j));
            matPoint[0] = MAX_COLOR_VALUE;
            matPoint[1] = MAX_COLOR_VALUE;
            matPoint[2] = MAX_COLOR_VALUE;
        }

        // Optionally add the robot marker to the image.
        if (request->include_robot_marker()) {
            const auto actualPose = currPose.params();

            std::feclearexcept(FE_ALL_EXCEPT);
            const auto j_float = widthScale * (actualPose[4] - minX);
            const auto i_float = heightScale * (actualPose[6] - minZ);
            if (std::fetestexcept(FE_OVERFLOW) ||
                std::fetestexcept(FE_UNDERFLOW)) {
                BOOST_LOG_TRIVIAL(debug)
                    << "Cannot scale robot marker point with X: "
                    << actualPose[4] << " and Z: " << actualPose[6]
                    << " to include on map with min X: " << minX
                    << ", min Z: " << minZ << ", widthScale: " << widthScale
                    << ", and heightScale: " << heightScale;
            } else if (i_float < 0 || i_float >= IMAGE_SIZE || j_float < 0 ||
                       j_float >= IMAGE_SIZE) {
                BOOST_LOG_TRIVIAL(debug)
                    << "Cannot include robot marker point with i: " << i_float
                    << " and j: " << j_float << " on map with image size "
                    << IMAGE_SIZE << "x" << IMAGE_SIZE;
            } else {
                const auto j = static_cast<int>(j_float);
                const auto i = static_cast<int>(i_float);
                cv::circle(mat, cv::Point(i, j), 5 /* radius */,
                           cv::Scalar(0, 0, MAX_COLOR_VALUE) /* red */,
                           cv::FILLED);
            }
        }

        // Encode the image as a jpeg.
        std::vector<uchar> buffer;
        try {
            cv::imencode(".jpeg", mat, buffer);
        } catch (std::exception &e) {
            std::ostringstream oss;
            oss << "error encoding image " << e.what();
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
        }

        // Write the image to the response.
        try {
            response->set_image(std::string(buffer.begin(), buffer.end()));
        } catch (std::exception &e) {
            std::ostringstream oss;
            oss << "error writing image to response " << e.what();
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
        }

    } else if (mime_type == "pointcloud/pcd") {
        // take sparse slam map and convert into a pcd. Orientation of PCD
        // is wrt the camera (z is coming out of the lens) so may need to
        // transform.

        std::stringbuf buffer;
        std::ostream oss(&buffer);

        // write our PCD file. we are writing as a binary
        oss << "VERSION .7\n"
            << "FIELDS x y z rgb\n"
            << "SIZE 4 4 4 4\n"
            << "TYPE F F F I\n"
            << "COUNT 1 1 1 1\n"
            << "WIDTH " << actualMap.size() << "\n"
            << "HEIGHT " << 1 << "\n"
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS " << actualMap.size() << "\n"
            << "DATA binary\n";

        // initial loop to determine color bounds for PCD.
        for (auto p : actualMap) {
            Eigen::Matrix<float, 3, 1> v = p->GetWorldPos();
            float val = v.y();
            if (max < val) max = val;
            if (min > val) min = val;
        }

        float mid = (max + min) / 2.;
        float span = max - min;
        int offsetRGB = 90;
        int spanRGB = 70;
        int clr = 0;
        cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::Mat valRGB2(hsv.size(), hsv.type());

        // write the map with simple rgb colors based off height from the
        // "ground". Map written as a binary
        for (auto p : actualMap) {
            Eigen::Matrix<float, 3, 1> v = p->GetWorldPos();
            float val = v.y();
            auto ratio = (val - mid) / span;

            clr = (int)(offsetRGB + (ratio * spanRGB));

            cv::Vec3b &color = hsv.at<cv::Vec3b>(cv::Point(0, 0));
            color[0] = clr;
            cv::cvtColor(hsv, valRGB2, cv::COLOR_HSV2RGB);
            cv::Vec3b colorRGB = valRGB2.at<cv::Vec3b>(cv::Point(0, 0));

            int rgb = 0;
            rgb = rgb | ((int)colorRGB[0] << 16);
            rgb = rgb | ((int)colorRGB[1] << 8);
            rgb = rgb | ((int)colorRGB[2] << 0);
            buffer.sputn((const char *)&v.x(), 4);
            buffer.sputn((const char *)&v.y(), 4);
            buffer.sputn((const char *)&v.z(), 4);
            buffer.sputn((const char *)&rgb, 4);
        }
        PointCloudObject *myPointCloud = response->mutable_point_cloud();
        myPointCloud->set_point_cloud(buffer.str());
    } else {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                            "mime_type should be \"image/jpeg\" or "
                            "\"pointcloud/pcd\", got \"" +
                                mime_type + "\"");
    }
    return grpc::Status::OK;
}

void SLAMServiceImpl::ProcessDataOnline(ORB_SLAM3::System *SLAM) {
    std::vector<std::string> filesRGB = utils::ListFilesInDirectoryForCamera(
        path_to_data + strRGB, ".png", camera_name);
    double fileTimeStart = yamlTime;
    // In online mode we want the most recent frames, so parse the data
    // directory with this in mind
    int locRecent = -1;
    locRecent = utils::FindFrameIndex(filesRGB, slam_mode, path_to_data,
                                      utils::FileParserMethod::Recent, yamlTime,
                                      &fileTimeStart);
    while (locRecent == -1) {
        if (!b_continue_session) return;
        BOOST_LOG_TRIVIAL(debug) << "No new files found";
        this_thread::sleep_for(frame_delay_msec);
        filesRGB = utils::ListFilesInDirectoryForCamera(path_to_data + strRGB,
                                                        ".png", camera_name);
        locRecent = utils::FindFrameIndex(filesRGB, slam_mode, path_to_data,
                                          utils::FileParserMethod::Recent,
                                          yamlTime, &fileTimeStart);
    }
    double timeStamp = 0, prevTimeStamp = 0, currTime = fileTimeStart;
    int i = locRecent;
    int nkeyframes = 0;

    while (true) {
        if (!b_continue_session) return;

        prevTimeStamp = timeStamp;
        // Look for new frames based off current timestamp
        // Currently pauses based off frame_delay_msec if no image is found
        while (i == -1) {
            if (!b_continue_session) return;
            filesRGB = utils::ListFilesInDirectoryForCamera(
                path_to_data + strRGB, ".png", camera_name);
            // In online mode we want the most recent frames, so parse the
            // data directorys with this in mind
            i = utils::FindFrameIndex(filesRGB, slam_mode, path_to_data,
                                      utils::FileParserMethod::Recent,
                                      prevTimeStamp + fileTimeStart, &currTime);
            if (i == -1) {
                this_thread::sleep_for(frame_delay_msec);
            } else {
                timeStamp = currTime - fileTimeStart;
            }
        }

        // decode images
        cv::Mat imRGB, imDepth;
        bool ok = false;
        if (slam_mode == "rgbd") {
            ok = utils::LoadRGBD(path_to_data, filesRGB[i], imRGB, imDepth);
        } else if (slam_mode == "mono") {
            ok = utils::LoadRGB(path_to_data, filesRGB[i], imRGB);
        } else {
            BOOST_LOG_TRIVIAL(fatal) << "Invalid slam_mode=" << slam_mode;
        }

        // Throw an error to skip this frame if it is not found
        if (!ok) {
            BOOST_LOG_TRIVIAL(error)
                << "Failed to load frame at: " << filesRGB[i];
        } else {
            // Pass the image to the SLAM system
            BOOST_LOG_TRIVIAL(debug) << "Passing image to SLAM";
            Sophus::SE3f tmpPose;
            if (slam_mode == "rgbd") {
                tmpPose = SLAM->TrackRGBD(imRGB, imDepth, timeStamp);
            } else if (slam_mode == "mono") {
                tmpPose = SLAM->TrackMonocular(imRGB, timeStamp);
            } else {
                BOOST_LOG_TRIVIAL(fatal) << "Invalid slam_mode=" << slam_mode;
            }
            // Update the copy of the current map whenever a change in
            // keyframes occurs
            ORB_SLAM3::Map *currMap = SLAM->GetAtlas()->GetCurrentMap();
            std::vector<ORB_SLAM3::KeyFrame *> keyframes =
                currMap->GetAllKeyFrames();
            {
                std::lock_guard<std::mutex> lock(slam_mutex);
                if (SLAM->GetTrackingState() ==
                    ORB_SLAM3::Tracking::eTrackingState::OK) {
                    poseGrpc = tmpPose.inverse();
                    if (nkeyframes != keyframes.size()) {
                        currMapPoints = currMap->GetAllMapPoints();
                    }
                }
            }
            BOOST_LOG_TRIVIAL(debug) << "Passed image to SLAM";
            nkeyframes = keyframes.size();
        }
        i = -1;
    }
    BOOST_LOG_TRIVIAL(info) << "Finished processing live images";
    return;
}

void SLAMServiceImpl::ProcessDataOffline(ORB_SLAM3::System *SLAM) {
    finished_processing_offline = false;
    // find all images used for our rgbd camera
    std::vector<std::string> filesRGB = utils::ListFilesInDirectoryForCamera(
        path_to_data + strRGB, ".png", camera_name);
    if (filesRGB.size() == 0) {
        BOOST_LOG_TRIVIAL(debug) << "No files found in " << strRGB;
        return;
    }

    double fileTimeStart = yamlTime, timeStamp = 0;
    // In offline mode we want the to parse all frames since our map/yaml
    // file was generated
    int locClosest = -1;
    locClosest = utils::FindFrameIndex(filesRGB, slam_mode, path_to_data,
                                       utils::FileParserMethod::Closest,
                                       yamlTime, &fileTimeStart);
    if (locClosest == -1) {
        BOOST_LOG_TRIVIAL(error) << "No new images to process in directory";
        return;
    }
    int nkeyframes = 0;

    // iterate over all remaining files in directory
    for (int i = locClosest; i < filesRGB.size(); i++) {
        //  record timestamp
        timeStamp = utils::ReadTimeFromFilename(filesRGB[i].substr(
                        filesRGB[i].find("_data_") + filenamePrefixLength)) -
                    fileTimeStart;
        // decode images
        cv::Mat imRGB, imDepth;
        bool ok = false;
        if (slam_mode == "rgbd") {
            ok = utils::LoadRGBD(path_to_data, filesRGB[i], imRGB, imDepth);
        } else if (slam_mode == "mono") {
            ok = utils::LoadRGB(path_to_data, filesRGB[i], imRGB);
        } else {
            BOOST_LOG_TRIVIAL(fatal) << "Invalid slam_mode=" << slam_mode;
        }
        // Throw an error to skip this frame if not found
        if (!ok) {
            BOOST_LOG_TRIVIAL(error)
                << "Failed to load frame at: " << filesRGB[i];
        } else {
            // Pass the image to the SLAM system
            BOOST_LOG_TRIVIAL(debug) << "Passing image to SLAM";

            Sophus::SE3f tmpPose;
            if (slam_mode == "rgbd") {
                tmpPose = SLAM->TrackRGBD(imRGB, imDepth, timeStamp);
            } else if (slam_mode == "mono") {
                tmpPose = SLAM->TrackMonocular(imRGB, timeStamp);
            } else {
                BOOST_LOG_TRIVIAL(fatal) << "Invalid slam_mode=" << slam_mode;
            }

            // Update the copy of the current map whenever a change in
            // keyframes occurs
            ORB_SLAM3::Map *currMap = SLAM->GetAtlas()->GetCurrentMap();
            std::vector<ORB_SLAM3::KeyFrame *> keyframes =
                currMap->GetAllKeyFrames();
            {
                std::lock_guard<std::mutex> lock(slam_mutex);
                if (SLAM->GetTrackingState() ==
                    ORB_SLAM3::Tracking::eTrackingState::OK) {
                    poseGrpc = tmpPose.inverse();
                    if (nkeyframes != keyframes.size()) {
                        currMapPoints = currMap->GetAllMapPoints();
                    }
                }
            }
            nkeyframes = keyframes.size();
        }
        if (!b_continue_session) break;
    }
    finished_processing_offline = true;
    BOOST_LOG_TRIVIAL(info) << "Finished processing offline images";
    return;
}

// Creates a simple map containing a 2x4x8 rectangular prism with the robot
// in the center, for testing GetMap and GetPosition.
void SLAMServiceImpl::ProcessDataForTesting(ORB_SLAM3::System *SLAM) {
    std::vector<Eigen::Vector3f> worldPos{{0, 0, 0}, {0, 0, 8}, {0, 4, 0},
                                          {0, 4, 8}, {2, 0, 0}, {2, 0, 8},
                                          {2, 4, 0}, {2, 4, 8}};
    std::vector<ORB_SLAM3::MapPoint> mapPoints(8);
    for (size_t i = 0; i < worldPos.size(); i++) {
        mapPoints[i].SetWorldPos(worldPos[i]);
    }

    Sophus::SO3f so3;
    Eigen::Vector3f translation{1, 2, 4};

    {
        std::lock_guard<std::mutex> lock(slam_mutex);
        currMapPoints.clear();
        for (auto &&p : mapPoints) {
            currMapPoints.push_back(&p);
        }
        poseGrpc = Sophus::SE3f(so3, translation);
    }
    BOOST_LOG_TRIVIAL(info) << "Finished creating map for testing";

    // Continue to serve requests.
    while (b_continue_session) {
        this_thread::sleep_for(
            chrono::microseconds(checkForShutdownIntervalMicroseconds));
    }
}

void SLAMServiceImpl::StartSaveAtlasAsOsa(ORB_SLAM3::System *SLAM) {
    if (map_rate_sec == chrono::seconds(0)) {
        return;
    }
    thread_save_atlas_as_osa_with_timestamp = new thread(
        [&](ORB_SLAM3::System *SLAM) {
            this->SaveAtlasAsOsaWithTimestamp(SLAM);
        },
        SLAM);
}

void SLAMServiceImpl::StopSaveAtlasAsOsa() {
    if (map_rate_sec == chrono::seconds(0)) {
        return;
    }
    thread_save_atlas_as_osa_with_timestamp->join();
}

void SLAMServiceImpl::SaveAtlasAsOsaWithTimestamp(ORB_SLAM3::System *SLAM) {
    auto check_for_shutdown_interval_usec =
        chrono::microseconds(checkForShutdownIntervalMicroseconds);
    while (b_continue_session) {
        auto start = std::chrono::high_resolution_clock::now();
        string path_save_file_name =
            utils::MakeFilenameWithTimestamp(path_to_map, camera_name);
        if (offlineFlag && finished_processing_offline) {
            {
                std::lock_guard<std::mutex> lock(slam_mutex);
                SLAM->SaveAtlasAsOsaWithTimestamp(path_save_file_name);
            }
            BOOST_LOG_TRIVIAL(debug) << "Finished saving final map";
            return;
        }
        if ((SLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames().size() !=
             0) &&
            (SLAM->GetTrackingState() ==
             ORB_SLAM3::Tracking::eTrackingState::OK)) {
            std::lock_guard<std::mutex> lock(slam_mutex);
            SLAM->SaveAtlasAsOsaWithTimestamp(path_save_file_name);
        }

        // Sleep for map_rate_sec duration, but check frequently for
        // shutdown
        while (b_continue_session) {
            std::chrono::duration<double, std::milli> time_elapsed_msec =
                std::chrono::high_resolution_clock::now() - start;
            if ((time_elapsed_msec >= map_rate_sec) ||
                (finished_processing_offline)) {
                break;
            }
            if (map_rate_sec - time_elapsed_msec >=
                check_for_shutdown_interval_usec) {
                this_thread::sleep_for(check_for_shutdown_interval_usec);
            } else {
                this_thread::sleep_for(map_rate_sec - time_elapsed_msec);
                break;
            }
        }
    }
}

namespace utils {
string time_format = "%Y-%m-%dT%H:%M:%SZ";
// LoadRGB loads in rgb images to be used by ORBSLAM, and
// returns whether the image was loaded successfully
bool LoadRGB(std::string path_to_data, std::string filename, cv::Mat &imRGB) {
    // write out the filename for the image
    std::string colorName = path_to_data + strRGB + "/" + filename + ".png";

    // check if the rgb image exists, if it does then load in the
    // image
    if (boost::filesystem::exists(colorName)) {
        imRGB = cv::imread(colorName, cv::IMREAD_COLOR);
        if (imRGB.empty()) return false;
        return true;
    }
    return false;
}

// LoadRGBD loads in a rgbd pair of images to be used by ORBSLAM, and
// returns whether the current pair is okay
bool LoadRGBD(std::string path_to_data, std::string filename, cv::Mat &imRGB,
              cv::Mat &imDepth) {
    // write out filenames and paths for each respective image
    std::string colorName = path_to_data + strRGB + "/" + filename + ".png";
    std::string depthName = path_to_data + strDepth + "/" + filename + ".png";

    // check if the rgb and depth image exists, if it does then load in the
    // images
    if (boost::filesystem::exists(colorName) &&
        boost::filesystem::exists(depthName)) {
        imRGB = cv::imread(colorName, cv::IMREAD_COLOR);
        imDepth = cv::imread(depthName, cv::IMREAD_UNCHANGED);
        if (imRGB.empty() || imDepth.empty()) return false;
        return true;
    }
    return false;
}

// find a specific input argument from rdk and write the value to a string.
// Returns empty if the argument is not found.
string ArgParser(const vector<string> &args, string strName) {
    // Possibly remove these in a future task
    string strVal;
    string currArg;
    size_t loc;
    for (auto &&currArg : args) {
        loc = currArg.find(strName);
        if (loc != string::npos) {
            strVal = currArg.substr(loc + strName.size());
            break;
        }
    }
    return strVal;
}

// parse a config map for a specific variable name and return the value as a
// string. Returns empty if the variable is not found within the map.
string ConfigMapParser(string map, string varName) {
    string strVal;
    size_t loc = string::npos;

    stringstream ss(map.substr(map.find("{") + 1, map.find("}") - 1));
    while (ss.good()) {
        string substr;
        getline(ss, substr, ',');
        loc = substr.find(varName);
        if (loc != string::npos) {
            strVal = substr.substr(loc + varName.size());
            break;
        }
    }

    return strVal;
}

void ParseAndValidateArguments(const vector<string> &args,
                               SLAMServiceImpl &slamService) {
    if (args.size() < 6) {
        throw runtime_error(
            "No args found. Expected: \n"
            "./bin/orb_grpc_server "
            "-data_dir=path_to_data "
            "-config_param={mode=slam_mode,} "
            "-port=grpc_port "
            "-sensors=sensor_name "
            "-data_rate_ms=frame_delay "
            "-map_rate_sec=map_rate_sec");
    }

    const auto config_params = ArgParser(args, "-config_param=");

    const auto debugParam = ConfigMapParser(config_params, "debug=");
    bool isDebugTrue;
    bool isDebugOne;
    istringstream(debugParam) >> std::boolalpha >> isDebugTrue;
    istringstream(debugParam) >> std::noboolalpha >> isDebugOne;
    if (!isDebugTrue && !isDebugOne) {
        boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                            boost::log::trivial::info);
    }

    for (auto i = 0; i < args.size(); i++) {
        BOOST_LOG_TRIVIAL(debug) << "Argument #" << i << " is " << args.at(i);
    }

    const auto data_dir = ArgParser(args, "-data_dir=");
    if (data_dir.empty()) {
        throw runtime_error("No data directory given");
    }
    slamService.path_to_vocab = data_dir + "/config/ORBvoc.txt";
    slamService.path_to_settings = data_dir + "/config";

    slamService.path_to_data = data_dir + "/data";
    slamService.path_to_map = data_dir + "/map";

    slamService.slam_mode = ConfigMapParser(config_params, "mode=");
    if (slamService.slam_mode.empty()) {
        throw runtime_error("No SLAM mode given");
    }
    boost::algorithm::to_lower(slamService.slam_mode);
    if (slamService.slam_mode != "rgbd" && slamService.slam_mode != "mono") {
        throw runtime_error("Invalid slam_mode=" + slamService.slam_mode);
    }

    slamService.slam_port = ArgParser(args, "-port=");
    if (slamService.slam_port.empty()) {
        throw runtime_error("No gRPC port given");
    }

    const auto data_rate_msec = ArgParser(args, "-data_rate_ms=");
    if (data_rate_msec.empty()) {
        throw runtime_error("No camera data rate specified");
    }
    slamService.frame_delay_msec = chrono::milliseconds(stoi(data_rate_msec));

    auto map_rate_sec = ArgParser(args, "-map_rate_sec=");
    if (map_rate_sec.empty()) {
        map_rate_sec = "0";
    }
    slamService.map_rate_sec = chrono::seconds(stoi(map_rate_sec));

    slamService.camera_name = ArgParser(args, "-sensors=");
    if (slamService.camera_name.empty()) {
        BOOST_LOG_TRIVIAL(info) << "No camera given -> running in offline mode";
        slamService.offlineFlag = true;
    }
    string local_viewer = ArgParser(args, "--localView=");
    boost::algorithm::to_lower(local_viewer);
    if ((local_viewer == "true") && (slamService.offlineFlag)) {
        BOOST_LOG_TRIVIAL(info) << "Running with local viewer";
        slamService.local_viewer_flag = true;
    } else {
        slamService.local_viewer_flag = false;
    }
}

// Converts UTC time string to a double value.
double ReadTimeFromFilename(string filename) {
    std::string::size_type sz;
    // Create a stream which we will use to parse the string
    std::istringstream ss(filename);

    // Create a tm object to store the parsed date and time.
    std::tm dt = {0};

    // Now we read from buffer using get_time manipulator
    // and formatting the input appropriately.
    ss >> std::get_time(&dt,time_format.c_str());
    time_t thisTime = std::mktime(&dt);
    auto sub_sec_index = filename.find(".");
    if ((sub_sec_index != string::npos)) {
        double sub_sec = (double)std::stof(filename.substr(sub_sec_index), &sz);
        double myTime = (double)thisTime + sub_sec;
        return myTime;
    } else {
        return (double)thisTime;
    }
}

// Checks if using new or old time format
void FindTimeFormat(string filename) {
    // Check if we are using the old time format
    auto find_underscores = filename.find("_");
    if(find_underscores != string::npos)
        time_format = "%Y-%m-%dT%H_%M_%SZ";
    return;
    
}

std::vector<std::string> ListFilesInDirectoryForCamera(
    std::string data_directory, std::string extension,
    std::string camera_name) {
    std::vector<std::string> file_paths;
    std::string currFile;
    for (const auto &entry : directory_iterator(data_directory)) {
        currFile = (entry.path()).stem().string();
        if (camera_name == currFile.substr(0, currFile.find("_data_"))) {
            file_paths.push_back(currFile);
        }
    }
    sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

// Find the next frame based off the current interest given a directory of
// data and a time to search from.
int FindFrameIndex(const std::vector<std::string> &filesRGB,
                   std::string slam_mode, std::string path_to_data,
                   FileParserMethod interest, double configTime,
                   double *timeInterest) {
    double fileTime;
    // Find the file closest to the configTime, used mostly in offline mode
    if (interest == FileParserMethod::Closest) {
        // for closest file, just parse the rgb directory. as LoadRGBD will
        // filter any MONOCULAR frames
        for (int i = 0; i < (int)filesRGB.size() - 1; i++) {
            fileTime = ReadTimeFromFilename(filesRGB[i].substr(
                filesRGB[i].find("_data_") + filenamePrefixLength));
            if (fileTime > configTime) {
                *timeInterest = fileTime;
                return i;
            }
        }
    }
    // Find the file generated most recently, used mostly in online mode
    else if (interest == FileParserMethod::Recent) {
        int i = (int)filesRGB.size() - 2;

        // if we have no files return -1 as an error
        if (i < 0) return -1;

        if (slam_mode == "mono") {
            fileTime = ReadTimeFromFilename(filesRGB[i].substr(
                filesRGB[i].find("_data_") + filenamePrefixLength));

            // if the latest file is older than our config time, return -1 as an
            // error
            if (fileTime <= configTime) return -1;

            *timeInterest = fileTime;
            return i;
        }

        if (slam_mode == "rgbd") {
            // for the most recent file, search the rgb directory until a
            // corresponding depth image is found
            std::string depthPath = path_to_data + strDepth + "/";
            for (i = (int)filesRGB.size() - 2; i >= 0; i--) {
                fileTime = ReadTimeFromFilename(filesRGB[i].substr(
                    filesRGB[i].find("_data_") + filenamePrefixLength));

                // if we found no new files return -1 as an error
                if (fileTime <= configTime) return -1;

                if (boost::filesystem::exists(depthPath + filesRGB[i] +
                                              ".png")) {
                    *timeInterest = fileTime;
                    return i;
                }
            }
        }
    }
    // if we do not find a file return -1 as an error
    return -1;
}

// Make a filename to a specific location for a sensor with a timestamp
// currently does not support millisecond resolution
// TODO change time format to .Format(time.RFC3339Nano)
// https://viam.atlassian.net/browse/DATA-277
string MakeFilenameWithTimestamp(string path_to_dir, string camera_name) {
    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), time_format.c_str(), std::gmtime(&t));
    // Save the current atlas map in *.osa style
    string path_save_file_name =
        path_to_dir + "/" + camera_name + "_data_" + timestamp + ".0000Z.osa";
    return path_save_file_name;
}

}  // namespace utils
}  // namespace viam
