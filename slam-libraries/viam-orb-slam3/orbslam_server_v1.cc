/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */
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
enum class FileParserMethod { Recent, Closest };
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
void exit_loop_handler(int s) {
    BOOST_LOG_TRIVIAL(info) << "Finishing session";
    b_continue_session = false;
}

void LoadImagesRGBD(const string &pathSeq, const string &strPathTimes,
                    vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD,
                    vector<double> &vTimeStamps);
string argParser(int argc, char **argv, const string varName);
string configMapParser(string map, string varName);
double readTimeFromFilename(const string filename);
std::vector<std::string> listFilesInDirectoryForCamera(
    const std::string data_directory, const std::string extension,
    const std::string camera_name);
void loadRGBD(std::string path_to_data, std::string filename, cv::Mat &im, cv::Mat &depth);
void decodeBOTH(std::string filename, cv::Mat &im, cv::Mat &depth);
int parseDataDir(const std::vector<std::string> &files,
                 FileParserMethod interest, double configTime,
                 double *timeInterest);

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
                // cout << v.x() << "  " << v.y() << "  " << v.z() << endl;
                buffer.sputn((const char *)&rgb, 4);
            }
            auto actualPose = poseGrpc.params();
            // cout << actualPose[4] << "  " << actualPose[5] << "  " << actualPose[6] << endl;
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
            listFilesInDirectoryForCamera(path_to_data + "/depth", ".png", camera_name);
        double fileTimeStart = yamlTime;
        // In online mode we want the most recent frames, so parse the data
        // directory with this in mind
        int locRecent = parseDataDir(files, FileParserMethod::Recent, yamlTime,
                                     &fileTimeStart);
        while (locRecent == -1) {
            if (!b_continue_session) return;
            BOOST_LOG_TRIVIAL(debug) << "No new files found";
            usleep(frame_delay * 1e3);
            files = listFilesInDirectoryForCamera(path_to_data + "/depth", ".png",
                                                  camera_name);
            locRecent = parseDataDir(files, FileParserMethod::Recent, yamlTime,
                                     &fileTimeStart);
        }

        double timeStamp = 0, prevTimeStamp = 0, currTime = fileTimeStart;
        int i = locRecent;
        int nkeyframes = 0;

        while (true) {
            // TBD: Possibly split this function into RGBD and MONO processing
            // modes
            //  https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-182
            if (!b_continue_session) return;

            prevTimeStamp = timeStamp;
            // Look for new frames based off current timestamp
            // Currently pauses based off frame_delay if no image is found
            while (i == -1) {
                if (!b_continue_session) return;
                files = listFilesInDirectoryForCamera(path_to_data + "/depth", ".png",
                                                      camera_name);
                // In online mode we want the most recent frames, so parse the
                // data directory with this in mind
                i = parseDataDir(files, FileParserMethod::Recent,
                                 prevTimeStamp + fileTimeStart, &currTime);
                if (i == -1) {
                    usleep(frame_delay * 1e3);
                } else {
                    timeStamp = currTime - fileTimeStart;
                }
            }
            // decode images
            cv::Mat im, depth;
            loadRGBD(path_to_data, files[i], im, depth);

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
            listFilesInDirectoryForCamera(path_to_data + "/rgb", ".png", camera_name);
        if (files.size() == 0) {
            BOOST_LOG_TRIVIAL(debug) << "No files found";
            return;
        }

        double fileTimeStart = yamlTime, timeStamp = 0;
        // In offline mode we want the to parse all frames since our map/yaml
        // file was generated
        int locClosest = parseDataDir(files, FileParserMethod::Closest,
                                      yamlTime, &fileTimeStart);
        if (locClosest == -1) {
            BOOST_LOG_TRIVIAL(error) << "No new images to process in directory";
            return;
        }
        int nkeyframes = 0;
        float imageScale = SLAM->GetImageScale();
        // iterate over all remaining files in directory
        for (int i = locClosest; i < files.size(); i++) {
            // TBD: Possibly split this function into RGBD and MONO processing
            // modes
            //  https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-182
            //  record timestamp
            timeStamp = readTimeFromFilename(files[i].substr(
                            files[i].find("_data_") + FILENAME_CONST)) -
                        fileTimeStart;
            // decode images
            cv::Mat im, depth;
            // loadRGBD(path_to_data, files[i], im, depth);
            // write out filenames and paths for each respective image
            std::string colorName = path_to_data + "/rgb/"  + files[i] + ".png";
            std::string depthName = path_to_data + "/depth/"  + files[i] + ".png";

            //check if the depth image exists, if it does then load in the images
            if (boost::filesystem::exists( colorName )){
                im = cv::imread(colorName, cv::IMREAD_UNCHANGED);
                depth = cv::imread(depthName, cv::IMREAD_GRAYSCALE);
            } 
            if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));
        }
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
                cout << "SLAMMIN " << depthName << endl;
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


    void process_rgbd_old_both(ORB_SLAM3::System *SLAM) {
        // find all images used for our rgbd camera
        std::vector<std::string> files =
            listFilesInDirectoryForCamera(path_to_data, ".both", camera_name);
        if (files.size() == 0) {
            BOOST_LOG_TRIVIAL(debug) << "No files found";
            return;
        }

        double fileTimeStart = yamlTime, timeStamp = 0;
        // In offline mode we want the to parse all frames since our map/yaml
        // file was generated
        int locClosest = parseDataDir(files, FileParserMethod::Closest,
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
            //  https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-182
            //  record timestamp
            timeStamp = readTimeFromFilename(files[i].substr(
                            files[i].find("_data_") + FILENAME_CONST)) -
                        fileTimeStart;
            // decode images
            cv::Mat im, depth;
            decodeBOTH(path_to_data + "/" + files[i], im, depth);
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
        LoadImagesRGBD(pathSeq, strAssociationFilename, vstrImageFilenamesRGB,
                       vstrImageFilenamesD, vTimestamps);
    cout << strAssociationFilename << endl;
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
            cout << vstrImageFilenamesD[ni] << endl;
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

    string path_to_data;
    string path_to_sequence;
    string camera_name;
    double yamlTime;
    int frame_delay;
    bool offlineFlag = false;

    std::mutex slam_mutex;
    Sophus::SE3f poseGrpc;
    std::vector<ORB_SLAM3::MapPoint *> currMapPoints;
};

int main(int argc, char **argv) {
    // TODO: change inputs to match args from rdk
    // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-179
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    if (argc < 6) {
        BOOST_LOG_TRIVIAL(fatal) << "No args found. Expected: \n"
                                 << "./bin/orb_grpc_server "
                                    "-data_dir=path_to_data "
                                    "-config_param={mode=slam_mode,} "
                                    "-port=grpc_port "
                                    "-sensors=sensor_name "
                                    "-data_rate_ms=frame_delay";
        return 1;
    }
    
    string config_params = argParser(argc, argv, "-config_param=");

    const auto debugParam = configMapParser(config_params, "debug=");
    bool isDebugTrue;
    bool isDebugOne;
    istringstream(debugParam) >> std::boolalpha >> isDebugTrue;
    istringstream(debugParam) >> std::noboolalpha >> isDebugOne;
    if (!isDebugTrue && !isDebugOne) {
        boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                            boost::log::trivial::info);
    }

    for (int i = 0; i < argc; ++i) {
        BOOST_LOG_TRIVIAL(debug) << "Argument #" << i << " is " << argv[i];
    }
    // setup the SLAM server
    SLAMServiceImpl slamService;
    ServerBuilder builder;

    string actual_path = argParser(argc, argv, "-data_dir=");
    if (actual_path.empty()) {
        BOOST_LOG_TRIVIAL(fatal) << "No data directory given";
        return 1;
    }
    string path_to_vocab = actual_path + "/config/ORBvoc.txt";
    string path_to_settings = actual_path + "/config";  // testORB.yaml";

    slamService.path_to_data =
        actual_path + "/data";  // will change in DATA 127/181

    // leaving commented for possible testing
    // string dummyPath = "/home/kat/johnstuff/slam/slam-libraries/viam-orb-slam3";
    // slamService.path_to_data = dummyPath + "/officePics6";
    // slamService.path_to_sequence = "Out_file.txt";
    string slam_mode = configMapParser(config_params, "mode=");
    if (slam_mode.empty()) {
        BOOST_LOG_TRIVIAL(fatal) << "No SLAM mode given";
        return 1;
    }

    string slam_port = argParser(argc, argv, "-port=");
    if (slam_port.empty()) {
        BOOST_LOG_TRIVIAL(fatal) << "No gRPC port given";
        return 1;
    }

    string frames = argParser(argc, argv, "-data_rate_ms=");
    if (frames.empty()) {
        BOOST_LOG_TRIVIAL(fatal) << "No camera data rate specified";
        return 1;
    }
    slamService.frame_delay = stoi(frames);

    slamService.camera_name = argParser(argc, argv, "-sensors=");
    if (slamService.camera_name.empty()) {
        BOOST_LOG_TRIVIAL(info) << "No camera given -> running in offline mode";
        slamService.offlineFlag = true;
    }

    builder.AddListeningPort(slam_port, grpc::InsecureServerCredentials());
    builder.RegisterService(&slamService);

    // Start the SLAM gRPC server
    std::unique_ptr<Server> server(builder.BuildAndStart());
    BOOST_LOG_TRIVIAL(info) << "Server listening on " << slam_port.c_str();

    // Determine which settings file to use(.yaml)
    const path myPath(path_to_settings);
    path latest;
    std::time_t latest_tm = 0;
    for (auto &&entry :
         boost::make_iterator_range(directory_iterator(myPath), {})) {
        path p = entry.path();

        if (is_regular_file(p) && p.extension() == ".yaml") {
            std::time_t timestamp = last_write_time(p);
            if (timestamp > latest_tm) {
                if (slamService.offlineFlag ||
                    p.stem().string().find(slamService.camera_name) !=
                        string::npos) {
                    latest = p;
                    latest_tm = timestamp;
                }
            }
        }
    }
    if (latest.empty()) {
        BOOST_LOG_TRIVIAL(fatal)
            << "No correctly formatted .yaml file found, Expected:\n"
               "{sensor}_data_{dateformat}.yaml";
        return 1;
    }

    // report the current yaml file check if it matches our format
    const string myYAML = latest.stem().string();
    BOOST_LOG_TRIVIAL(debug) << "Our yaml file: " << myYAML;
    string full_path_to_settings =
        path_to_settings + "/" + latest.filename().string();
    if (slamService.offlineFlag) {
        if (myYAML.find("_data_") != string::npos)
            slamService.camera_name = myYAML.substr(0, myYAML.find("_data_"));
        else {
            BOOST_LOG_TRIVIAL(fatal)
                << "No correctly formatted .yaml file found, Expected:\n"
                   "{sensor}_data_{dateformat}.yaml\n"
                   "as most the recent config in directory";
            return 1;
        }
    }

    // Grab timestamp from yaml
    slamService.yamlTime = readTimeFromFilename(
        myYAML.substr(myYAML.find("_data_") + FILENAME_CONST));
    BOOST_LOG_TRIVIAL(debug)
        << "The time from our config is: " << slamService.yamlTime
        << " seconds";

    // Start SLAM
    SlamPtr SLAM = nullptr;
    boost::algorithm::to_lower(slam_mode);

    if (slam_mode == "rgbd") {
        BOOST_LOG_TRIVIAL(info) << "RGBD selected";

        // Create SLAM system. It initializes all system threads and gets ready
        // to process frames.
        // SLAM = std::make_unique<ORB_SLAM3::System>(
        //     path_to_vocab, full_path_to_settings, ORB_SLAM3::System::MONOCULAR,
        //     false, 0);
        SLAM = std::make_unique<ORB_SLAM3::System>(
            path_to_vocab, full_path_to_settings, ORB_SLAM3::System::RGBD,
            false, 0);
        if (slamService.offlineFlag) {
            BOOST_LOG_TRIVIAL(info) << "Running in offline mode";
            slamService.process_rgbd_offline(SLAM.get());
            // slamService.process_rgbd_old(SLAM);
            // Continue to serve requests.
            while (b_continue_session) {
                usleep(CHECK_FOR_SHUTDOWN_INTERVAL);
            }
        } else {
            BOOST_LOG_TRIVIAL(info) << "Running in online mode";
            slamService.process_rgbd_online(SLAM.get());
        }
        // slamService.process_rgbd_old(SLAM);
        // slamService.process_rgbd_for_testing(SLAM.get());

    } else if (slam_mode == "mono") {
        // TODO implement MONO
        // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-182
        // SLAM = std::make_unique<ORB_SLAM3::System>(
        //     path_to_vocab, full_path_to_settings, ORB_SLAM3::System::MONOCULAR,
        //     false, 0);
        // auto tmpPose = SLAM->TrackMonocular(im,timeStamp);
    } else {
        BOOST_LOG_TRIVIAL(fatal) << "Invalid slam_mode=" << slam_mode;
        return 1;
    }
    
    SLAM->Shutdown();
    BOOST_LOG_TRIVIAL(info) << "System shutdown";

    return 0;
}

// this function loads in a rgbd pair of images to be used by ORBSLAM
void loadRGBD(std::string path_to_data, std::string filename, cv::Mat &im, cv::Mat &depth) {
    // write out filenames and paths for each respective image
    std::string colorName = path_to_data + "/rgb/"  + filename + ".png";
    std::string depthName = path_to_data + "/depth/"  + filename + ".png";

    //check if the depth image exists, if it does then load in the images
    if (boost::filesystem::exists( colorName )){
        im = cv::imread(colorName, cv::IMREAD_UNCHANGED);
        depth = cv::imread(depthName, cv::IMREAD_UNCHANGED);
    } 
    return;

}

void LoadImagesRGBD(const string &pathSeq, const string &strPathTimes,
                    vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD,
                    vector<double> &vTimeStamps) {
    // This function will be removed in a future update. Currently this is only
    // used with a previous dataset
    string pathCam0 = pathSeq + "/rgb";
    string pathCam1 = pathSeq + "/depth";
    std::ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageFilenamesRGB.reserve(5000);
    vstrImageFilenamesD.reserve(5000);
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            vstrImageFilenamesRGB.push_back(pathCam0 + "/" + ss.str());
            vstrImageFilenamesD.push_back(pathCam1 + "/" + ss.str());
            string timestring = s.substr(0, s.find_last_of("."));
            std::string::size_type sz;
            double t = std::stod(timestring, &sz);
            vTimeStamps.push_back(t);
        }
    }
}

// find a specific input argument from rdk and write the value to a string.
// Returns empty if the argument is not found.
string argParser(int argc, char **argv, string strName) {
    // Possibly remove these in a future task
    string strVal;
    string currArg;
    size_t loc;
    for (int i = 0; i < argc; ++i) {
        currArg = string(argv[i]);
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
string configMapParser(string map, string varName) {
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

// Converts UTC time string to a double value.
double readTimeFromFilename(string filename) {
    std::string::size_type sz;
    // Create a stream which we will use to parse the string
    std::istringstream ss(filename);

    // Create a tm object to store the parsed date and time.
    std::tm dt = {0};

    // Now we read from buffer using get_time manipulator
    // and formatting the input appropriately.
    ss >> std::get_time(&dt, "%Y-%m-%dT%H_%M_%SZ");
    double sub_sec =
        (double)std::stof(filename.substr(filename.find(".")), &sz);
    time_t thisTime = std::mktime(&dt);

    double myTime = (double)thisTime + sub_sec;
    return myTime;
}

std::vector<std::string> listFilesInDirectoryForCamera(
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

// take .both files from rdk and process them to use with ORBSLAM. this will be
// changed in
// https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-254

void decodeBOTH(std::string filename, cv::Mat &im, cv::Mat &depth) {
    cv::Mat rawData;
    std::ifstream fin(filename + ".both");
    if (fin.peek() == std::ifstream::traits_type::eof()) {
        BOOST_LOG_TRIVIAL(error) << "Bad file, found EOF";
        return;
    }
    std::vector<char> contents((std::istreambuf_iterator<char>(fin)),
                               std::istreambuf_iterator<char>());
    char *buffer = &contents[0];
    int frameWidth;
    int frameHeight;
    int location = 0;
    long j;
    int nSize = contents.size();

    // exit if no frame width or height is found
    if (nSize < sizeof(__int64_t) * 2) return;
    memcpy(&j, buffer + location, sizeof(__int64_t));
    location = location + sizeof(__int64_t);
    frameWidth = (int)j;

    memcpy(&j, buffer + location, sizeof(__int64_t));
    location = location + sizeof(__int64_t);
    frameHeight = (int)j;
    // exit if depth map is not complete(assumes 8bit)
    if (nSize < (8 * frameWidth * frameHeight)) return;

    int depthFrame[frameWidth][frameHeight];
    // copy depth map into 2D vector
    for (int x = 0; x < frameWidth; x++) {
        for (int y = 0; y < frameHeight; y++) {
            memcpy(&j, buffer + location, sizeof(__int64_t));
            depthFrame[x][y] = (int)j;
            location = location + sizeof(__int64_t);
        }
    }
    if (nSize <= location) {
        return;
    }
    // grab location of the png(occurs after depth map completes)
    char *pngBuf = &contents[location];

    // format frames with opencv
    depth = cv::Mat(cv::Size(frameWidth, frameHeight), CV_16U, depthFrame,
                    cv::Mat::AUTO_STEP);
    rawData = cv::Mat(cv::Size(1, nSize - location), CV_8UC1, (void *)pngBuf,
                      cv::IMREAD_COLOR);
    im = cv::imdecode(rawData, cv::IMREAD_COLOR);
}

// Find the next frame based off the current interest given a directory of
// data and time to search from
int parseDataDir(const std::vector<std::string> &files,
                 FileParserMethod interest, double configTime,
                 double *timeInterest) {
    // Find the file closest to the configTime, used mostly in offline mode
    if (interest == FileParserMethod::Closest) {
        for (int i = 0; i < files.size() - 1; i++) {
            double fileTime = readTimeFromFilename(
                files[i].substr(files[i].find("_data_") + FILENAME_CONST));
            double delTime = fileTime - configTime;
            if (delTime > 0) {
                *timeInterest = fileTime;
                return i;
            }
        }
    }
    // Find the file generated most recently, used mostly in online mode
    else if (interest == FileParserMethod::Recent) {
        int i = files.size() - 2;
        double fileTime = readTimeFromFilename(
            files[i].substr(files[i].find("_data_") + FILENAME_CONST));
        double delTime = fileTime - configTime;
        if (delTime > 0) {
            *timeInterest = fileTime;
            return i;
        }
    }
    // if we do not find a file return -1 as an error
    return -1;
}
