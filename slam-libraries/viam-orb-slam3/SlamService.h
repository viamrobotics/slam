#include "Utils.h"
#include <System.h>
#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <signal.h>
#include <algorithm>
#include <atomic>
#include <cfenv>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <sstream>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>

#include "proto/api/common/v1/common.grpc.pb.h"
#include "proto/api/common/v1/common.pb.h"
#include "proto/api/service/slam/v1/slam.grpc.pb.h"
#include "proto/api/service/slam/v1/slam.pb.h"

#pragma STDC FENV_ACCESS ON
#define _USE_MATH_DEFINES
using namespace std;
using namespace boost::filesystem;
#define FILENAME_CONST 6
#define IMAGE_SIZE 300
#define CHECK_FOR_SHUTDOWN_INTERVAL 1e5
#define MAX_COLOR_VALUE 255
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;

using proto::api::common::v1::PointCloudObject;
using proto::api::common::v1::Pose;
using proto::api::common::v1::PoseInFrame;
using proto::api::service::slam::v1::GetMapRequest;
using proto::api::service::slam::v1::GetMapResponse;
using proto::api::service::slam::v1::GetPositionRequest;
using proto::api::service::slam::v1::GetPositionResponse;
using proto::api::service::slam::v1::SLAMService;
using SlamPtr = std::unique_ptr<ORB_SLAM3::System>;

std::atomic<bool> b_continue_session{true};

#ifndef SLAM_SERVICE_H
#define SLAM_SERVICE_H


namespace viam {

class SLAMServiceImpl final : public SLAMService::Service {
   public:

    ::grpc::Status GetPosition(ServerContext *context,
                                const GetPositionRequest *request,
                                GetPositionResponse *response) override {
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

        // pull out pose into a vector [qx qy qz qw x y z] and transform into
        // angle axis + x y z. NOTE the origin of the pose is wrt the camera(z
        // axis comes out of the lense) so may require an additional
        // transformation
        double o_x, o_y, o_z;
        auto actualPose = currPose.params();
        float angle_rad = 2 * acos(actualPose[3]);
        double angle_deg = angle_rad * 180.0 / M_PI;
        myPose->set_theta(angle_deg);

        if (fmod(angle_rad, M_PI) != 0) {
            o_x = actualPose[0] / sin(angle_rad / 2.0);
            o_y = actualPose[1] / sin(angle_rad / 2.0);
            o_z = actualPose[2] / sin(angle_rad / 2.0);

        } else {
            o_x = actualPose[0];
            o_y = actualPose[1];
            o_z = actualPose[2];
        }

        // set pose for our response
        myPose->set_o_x(o_x);
        myPose->set_o_y(o_y);
        myPose->set_o_z(o_z);
        myPose->set_x(actualPose[4]);
        myPose->set_y(actualPose[5]);
        myPose->set_z(actualPose[6]);
        return grpc::Status::OK;
    }

    ::grpc::Status GetMap(ServerContext *context, const GetMapRequest *request,
                            GetMapResponse *response) override {
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
            for (auto &&p : actualMap) {
                const auto v = p->GetWorldPos();
                minX = std::min(minX, v.x());
                maxX = std::max(maxX, v.x());
                minZ = std::min(minZ, v.z());
                maxZ = std::max(maxZ, v.z());
            }
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
            if (std::fetestexcept(FE_OVERFLOW) ||
                std::fetestexcept(FE_UNDERFLOW)) {
                std::ostringstream oss;
                oss << "cannot create image from map with min X: " << minX
                    << ", max X: " << maxX << ", min Z: " << minZ
                    << ", and max Z: " << maxZ;
                return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
            }

            if (originalWidth <= 0 || originalHeight <= 0) {
                std::ostringstream oss;
                oss << "cannot create image from map with width: "
                    << originalWidth << " and height: " << originalHeight;
                return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
            }

            // Scale to constant size image.
            std::feclearexcept(FE_ALL_EXCEPT);
            const auto widthScale = (IMAGE_SIZE - 2) / originalWidth;
            const auto heightScale = (IMAGE_SIZE - 2) / originalHeight;
            if (std::fetestexcept(FE_OVERFLOW) ||
                std::fetestexcept(FE_UNDERFLOW)) {
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
                    return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                        oss.str());
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
                } else if (i_float < 0 || i_float >= IMAGE_SIZE ||
                            j_float < 0 || j_float >= IMAGE_SIZE) {
                    BOOST_LOG_TRIVIAL(debug)
                        << "Cannot include robot marker point with i: "
                        << i_float << " and j: " << j_float
                        << " on map with image size " << IMAGE_SIZE << "x"
                        << IMAGE_SIZE;
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
            float span = max - min;
            char clr = 0;
            int offsetRGB = 60;
            int spanRGB = 192;

            // write the map with simple rgb colors based off height from the
            // "ground". Map written as a binary
            for (auto p : actualMap) {
                Eigen::Matrix<float, 3, 1> v = p->GetWorldPos();
                float val = v.y();
                auto ratio = (val - min) / span;
                clr = (char)(offsetRGB + (ratio * spanRGB));
                if (clr > MAX_COLOR_VALUE) clr = MAX_COLOR_VALUE;
                if (clr < 0) clr = 0;
                int rgb = 0;
                rgb = rgb | (clr << 16);
                rgb = rgb | (clr << 8);
                rgb = rgb | (clr << 0);
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

    void process_rgbd_online(ORB_SLAM3::System *SLAM) {
        std::vector<std::string> files =
            viam::utils::listFilesInDirectoryForCamera(path_to_data, ".both", camera_name);
        double fileTimeStart = yamlTime;
        // In online mode we want the most recent frames, so parse the data
        // directory with this in mind
        int locRecent = viam::utils::parseDataDir(files, viam::utils::FileParserMethod::Recent, yamlTime,
                                        &fileTimeStart);
        while (locRecent == -1) {
            if (!b_continue_session) return;
            BOOST_LOG_TRIVIAL(debug) << "No new files found";
            usleep(frame_delay * 1e3);
            files = viam::utils::listFilesInDirectoryForCamera(path_to_data, ".both",
                                                    camera_name);
            locRecent = viam::utils::parseDataDir(files, viam::utils::FileParserMethod::Recent, yamlTime,
                                        &fileTimeStart);
        }

        double timeStamp = 0, prevTimeStamp = 0, currTime = fileTimeStart;
        int i = locRecent;
        int nkeyframes = 0;

        while (true) {
            // TBD: Possibly split this function into RGBD and MONO processing
            // modes
            // https://viam.atlassian.net/browse/DATA-182
            if (!b_continue_session) return;

            prevTimeStamp = timeStamp;
            // Look for new frames based off current timestamp
            // Currently pauses based off frame_delay if no image is found
            while (i == -1) {
                if (!b_continue_session) return;
                files = viam::utils::listFilesInDirectoryForCamera(path_to_data, ".both",
                                                        camera_name);
                // In online mode we want the most recent frames, so parse the
                // data directory with this in mind
                i = viam::utils::parseDataDir(files, viam::utils::FileParserMethod::Recent,
                                    prevTimeStamp + fileTimeStart, &currTime);
                if (i == -1) {
                    usleep(frame_delay * 1e3);
                } else {
                    timeStamp = currTime - fileTimeStart;
                }
            }
            // decode images
            cv::Mat im, depth;
            viam::utils::decodeBOTH(path_to_data + "/" + files[i], im, depth);

            // Throw an error to skip this frame if the frames are bad
            if (depth.empty()) {
                BOOST_LOG_TRIVIAL(error)
                    << "Failed to load depth at: " << files[i];
            } else if (im.empty()) {
                BOOST_LOG_TRIVIAL(error)
                    << "Failed to load png image at: " << files[i];
            } else {
                // Pass the image to the SLAM system
                BOOST_LOG_TRIVIAL(debug) << "Passing image to SLAM";
                auto tmpPose = SLAM->TrackRGBD(im, depth, timeStamp);
                // Update the copy of the current map whenever a change in
                // keyframes occurs
                ORB_SLAM3::Map *currMap = SLAM->GetAtlas()->GetCurrentMap();
                std::vector<ORB_SLAM3::KeyFrame *> keyframes =
                    currMap->GetAllKeyFrames();
                {
                    std::lock_guard<std::mutex> lock(slam_mutex);
                    poseGrpc = tmpPose;
                    if (SLAM->GetTrackingState() ==
                            ORB_SLAM3::Tracking::eTrackingState::OK &&
                        nkeyframes != keyframes.size()) {
                        currMapPoints = currMap->GetAllMapPoints();
                    }
                }
                nkeyframes = keyframes.size();
            }
            i = -1;
        }
        BOOST_LOG_TRIVIAL(info) << "Finished processing live images";
        return;
    }

    void process_rgbd_offline(ORB_SLAM3::System *SLAM) {
        // find all images used for our rgbd camera
        std::vector<std::string> files =
            viam::utils::listFilesInDirectoryForCamera(path_to_data, ".both", camera_name);
        if (files.size() == 0) {
            BOOST_LOG_TRIVIAL(debug) << "No files found";
            return;
        }

        double fileTimeStart = yamlTime, timeStamp = 0;
        // In offline mode we want the to parse all frames since our map/yaml
        // file was generated
        int locClosest = viam::utils::parseDataDir(files, viam::utils::FileParserMethod::Closest,
                                        yamlTime, &fileTimeStart);
        if (locClosest == -1) {
            BOOST_LOG_TRIVIAL(error) << "No new images to process in directory";
            return;
        }
        int nkeyframes = 0;

        // iterate over all remaining files in directory
        for (int i = locClosest; i < files.size(); i++) {
            // TBD: Possibly split this function into RGBD and MONO processing
            // modes
            // https://viam.atlassian.net/browse/DATA-182
            //  record timestamp
            timeStamp = viam::utils::readTimeFromFilename(files[i].substr(
                            files[i].find("_data_") + FILENAME_CONST)) -
                        fileTimeStart;
            // decode images
            cv::Mat im, depth;
            viam::utils::decodeBOTH(path_to_data + "/" + files[i], im, depth);
            // Throw an error to skip this frame if not found
            if (depth.empty()) {
                BOOST_LOG_TRIVIAL(error)
                    << "Failed to load depth at: " << files[i];
            } else if (im.empty()) {
                BOOST_LOG_TRIVIAL(error)
                    << "Failed to load png image at: " << files[i];
            } else {
                // Pass the image to the SLAM system
                BOOST_LOG_TRIVIAL(debug) << "Passing image to SLAM";
                auto tmpPose = SLAM->TrackRGBD(im, depth, timeStamp);

                // Update the copy of the current map whenever a change in
                // keyframes occurs
                ORB_SLAM3::Map *currMap = SLAM->GetAtlas()->GetCurrentMap();
                std::vector<ORB_SLAM3::KeyFrame *> keyframes =
                    currMap->GetAllKeyFrames();
                {
                    std::lock_guard<std::mutex> lock(slam_mutex);
                    poseGrpc = tmpPose;
                    if (SLAM->GetTrackingState() ==
                            ORB_SLAM3::Tracking::eTrackingState::OK &&
                        nkeyframes != keyframes.size()) {
                        currMapPoints = currMap->GetAllMapPoints();
                    }
                }
                nkeyframes = keyframes.size();
            }
            if (!b_continue_session) break;
        }

        BOOST_LOG_TRIVIAL(info) << "Finished processing offline images";
        return;
    }

    int process_rgbd_old(std::unique_ptr<ORB_SLAM3::System> &SLAM) {
        // Function used for ORB_SLAM with an rgbd camera. Currently returns int
        // as a placeholder for error signals should the server have to restart
        // itself.

        // This function will be removed in a future update. Currently this is
        // only used with a previous dataset

        int nImages = 0;
        int nkeyframes = 0;

        // Retrieve paths to images
        vector<string> vstrImageFilenamesRGB;
        vector<string> vstrImageFilenamesD;
        vector<double> vTimestamps;
        string strAssociationFilename =
            string(path_to_data) + "/" + string(path_to_sequence);
        string pathSeq(path_to_data);
        viam::utils::LoadImagesRGBD(pathSeq, strAssociationFilename, vstrImageFilenamesRGB,
                        vstrImageFilenamesD, vTimestamps);

        // Check consistency in the number of images and depthmaps
        nImages = vstrImageFilenamesRGB.size();
        if (vstrImageFilenamesRGB.empty()) {
            BOOST_LOG_TRIVIAL(error) << "No images found in provided path";
            return 1;
        } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
            BOOST_LOG_TRIVIAL(error)
                << "Different number of images for rgb and depth";
            return 1;
        }

        // Main loop
        cv::Mat imRGB, imD;
        for (int ni = 0; ni < nImages; ni++) {
            // Read image and depthmap from file
            imRGB = cv::imread(vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
            imD = cv::imread(vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);
            double tframe = vTimestamps[ni];

            if (imRGB.empty()) {
                BOOST_LOG_TRIVIAL(error)
                    << "Failed to load image at: " << vstrImageFilenamesRGB[ni];
                return 1;
            }

            // Pass the image to the SLAM system
            auto tmpPose = SLAM->TrackRGBD(imRGB, imD, tframe);

            // Update the copy of the current map whenever a change in keyframes
            // occurs
            ORB_SLAM3::Map *currMap = SLAM->GetAtlas()->GetCurrentMap();
            std::vector<ORB_SLAM3::KeyFrame *> keyframes =
                currMap->GetAllKeyFrames();
            {
                std::lock_guard<std::mutex> lock(slam_mutex);
                poseGrpc = tmpPose;
                if (SLAM->GetTrackingState() ==
                        ORB_SLAM3::Tracking::eTrackingState::OK &&
                    nkeyframes != keyframes.size()) {
                    currMapPoints = currMap->GetAllMapPoints();
                }
            }
            nkeyframes = keyframes.size();
            if (!b_continue_session) break;
        }

        BOOST_LOG_TRIVIAL(info) << "Finished processing images";

        return 1;
    }

    // Creates a simple map containing a 2x4x8 rectangular prism with the robot
    // in the center, for testing GetMap and GetPosition.
    void process_rgbd_for_testing(ORB_SLAM3::System *SLAM) {
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
            usleep(CHECK_FOR_SHUTDOWN_INTERVAL);
        }
    }

    void start_save_atlas_as_osa(ORB_SLAM3::System *SLAM, string path_save_atlas) {
        thread_save_atlas_as_osa_with_timestamp = new thread(
            [&](ORB_SLAM3::System *SLAM, string path_save_atlas) {
                this->save_atlas_as_osa_with_timestamp(SLAM, path_save_atlas);
            }, SLAM, path_save_atlas);
    }

    void stop_save_atlas_as_osa() {
        thread_save_atlas_as_osa_with_timestamp->join();
    }

    string path_to_data;
    string path_to_sequence;
    string camera_name;
    double yamlTime;
    int frame_delay;
    bool offlineFlag = false;
    int map_rate_sec;

    std::mutex slam_mutex;
    Sophus::SE3f poseGrpc;
    std::vector<ORB_SLAM3::MapPoint *> currMapPoints;

   private:

    std::thread* thread_save_atlas_as_osa_with_timestamp;

    void save_atlas_as_osa_with_timestamp(ORB_SLAM3::System *SLAM, string path_save_atlas) {
        while(b_continue_session) {
            std::time_t t = std::time(nullptr);
            char timestamp[100];
            std::strftime(timestamp, sizeof(timestamp), "%FT%H_%M_%S", std::gmtime(&t));
            // Save the current atlas map in *.osa style
            string pathSaveFileName = path_save_atlas;
            pathSaveFileName = pathSaveFileName.append("atlas_");
            pathSaveFileName = pathSaveFileName.append(timestamp);
            pathSaveFileName = pathSaveFileName.append(".osa");
            {
                std::lock_guard<std::mutex> lock(slam_mutex);
                SLAM->SaveAtlasAsOsaWithTimestamp(pathSaveFileName);
            }
            this_thread::sleep_for(chrono::seconds(map_rate_sec));
        }
    }
    };

}

#endif // SLAM_SERVICE_H