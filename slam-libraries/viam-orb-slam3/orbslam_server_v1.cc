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

#include "proto/api/common/v1/common.grpc.pb.h"
#include "proto/api/common/v1/common.pb.h"
#include "proto/api/service/slam/v1/slam.grpc.pb.h"
#include "proto/api/service/slam/v1/slam.pb.h"
#define _USE_MATH_DEFINES
using namespace std;
using namespace boost::filesystem;
#define FILENAME_CONST 6
enum FileParserMethod { Recent, Closest };
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

std::atomic<bool> b_continue_session{false};
void exit_loop_handler(int s) {
    cout << "Finishing session" << endl;
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

        if (mime_type == "image/jpeg") {
            // TODO: determine how to make 2D map
            // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-131

        } else if (mime_type == "image/pcd") {
            // take sparse slam map and convert into a pcd. Orientation of PCD
            // is wrt the camera (z is coming out of the lens) so may need to
            // transform.
            std::vector<ORB_SLAM3::MapPoint *> actualMap;
            {
                std::lock_guard<std::mutex> lk(slam_mutex);
                actualMap = currMapPoints;
            }
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
                if (clr > 255) clr = 255;
                if (clr < 0) clr = 0;
                int rgb = 0;
                rgb = rgb | (clr << 16);
                rgb = rgb | (clr << 8);
                rgb = rgb | (clr << 0);
                buffer.sputn((const char *)&v.x(), 4);
                buffer.sputn((const char *)&v.y(), 4);
                buffer.sputn((const char *)&rgb, 4);
            }
            PointCloudObject *myPointCloud = response->mutable_point_cloud();
            myPointCloud->set_point_cloud(buffer.str());
        }
        return grpc::Status::OK;
    }

    void process_rgbd_online(ORB_SLAM3::System *SLAM) {
        std::vector<std::string> files =
            listFilesInDirectoryForCamera(path_to_data, ".both", camera_name);
        double fileTimeStart = yamlTime;
        int locRecent = parseDataDir(files, Recent, yamlTime, &fileTimeStart);
        while ((locRecent == -1) && (b_continue_session)) {
            cout << "No new files found" << endl;
            usleep(frame_delay * 1e3);
            files = listFilesInDirectoryForCamera(path_to_data, ".both",
                                                  camera_name);
            locRecent = parseDataDir(files, Recent, yamlTime, &fileTimeStart);
        }
        if (!b_continue_session) return;

        double timeStamp = 0, prevTimeStamp = 0, currTime = fileTimeStart;
        int i = locRecent;
        int nkeyframes = 0;

        while (b_continue_session) {
            std::lock_guard<std::mutex> lock(slam_mutex);
            prevTimeStamp = timeStamp;
            // Look for new frames based off current timestamp
            // Currently pauses based off frame_delay if no image is found
            while ((i == -1) && (b_continue_session)) {
                files = listFilesInDirectoryForCamera(path_to_data, ".both",
                                                      camera_name);
                i = parseDataDir(files, Recent, prevTimeStamp + fileTimeStart,
                                 &currTime);
                if (i == -1) {
                    cerr << "No new frames found " << endl;
                    usleep(frame_delay * 1e3);
                } else {
                    timeStamp = currTime - fileTimeStart;
                }
            }
            if (!b_continue_session) return;
            // decode images
            cv::Mat im, depth;
            decodeBOTH(path_to_data + "/" + files[i], im, depth);

            // Throw an error to skip this frame if the frames are bad
            if (depth.empty()) {
                cerr << endl << "Failed to load depth at: " << files[i] << endl;
            } else if (im.empty()) {
                cerr << endl
                     << "Failed to load png image at: " << files[i] << endl;
            } else {
                // Pass the image to the SLAM system
                cout << "SLAMMIN" << endl;
                poseGrpc = SLAM->TrackRGBD(im, depth, timeStamp);
                // Update the copy of the current map whenever a change in
                // keyframes occurs
                ORB_SLAM3::Map *currMap = SLAM->GetAtlas()->GetCurrentMap();
                std::vector<ORB_SLAM3::KeyFrame *> keyframes =
                    currMap->GetAllKeyFrames();
                if (SLAM->GetTrackingState() ==
                        ORB_SLAM3::Tracking::eTrackingState::OK &&
                    nkeyframes != keyframes.size()) {
                    currMapPoints = currMap->GetAllMapPoints();
                }
                nkeyframes = keyframes.size();
            }
            i = -1;
        }
        cout << "Finished Processing Live Images\n" << endl;
        return;
    }

    void process_rgbd_offline(ORB_SLAM3::System *SLAM) {
        // find all images used for our rgbd camera
        std::vector<std::string> files =
            listFilesInDirectoryForCamera(path_to_data, ".both", camera_name);
        if (files.size() == 0) {
            cout << "no files found" << endl;
            return;
        }

        double fileTimeStart = yamlTime, timeStamp = 0;
        int locClosest = parseDataDir(files, Closest, yamlTime, &fileTimeStart);
        if (locClosest == -1) {
            cerr << "No new images to process in directory" << endl;
            return;
        }
        int nkeyframes = 0;

        // iterate over all remaining files in directory
        for (int i = locClosest; i < files.size(); i++) {
            std::lock_guard<std::mutex> lock(slam_mutex);
            // record timestamp
            timeStamp = readTimeFromFilename(files[i].substr(
                            files[i].find("_data_") + FILENAME_CONST)) -
                        fileTimeStart;
            // decode images
            cv::Mat im, depth;
            decodeBOTH(path_to_data + "/" + files[i], im, depth);
            // Throw an error to skip this frame if not found
            if (depth.empty()) {
                cerr << endl << "Failed to load depth at: " << files[i] << endl;
            } else if (im.empty()) {
                cerr << endl
                     << "Failed to load png image at: " << files[i] << endl;
            } else {
                // Pass the image to the SLAM system
                cout << "SLAMMIN  " << endl;
                poseGrpc = SLAM->TrackRGBD(im, depth, timeStamp);

                // Update the copy of the current map whenever a change in
                // keyframes occurs
                ORB_SLAM3::Map *currMap = SLAM->GetAtlas()->GetCurrentMap();
                std::vector<ORB_SLAM3::KeyFrame *> keyframes =
                    currMap->GetAllKeyFrames();
                if (SLAM->GetTrackingState() ==
                        ORB_SLAM3::Tracking::eTrackingState::OK &&
                    nkeyframes != keyframes.size()) {
                    currMapPoints = currMap->GetAllMapPoints();
                }
                nkeyframes = keyframes.size();
            }
            if (!b_continue_session) break;
        }

        cout << "Finished Processing Offline Images\n" << endl;
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

        // Check consistency in the number of images and depthmaps
        nImages = vstrImageFilenamesRGB.size();
        if (vstrImageFilenamesRGB.empty()) {
            cerr << endl << "No images found in provided path." << endl;
            return 1;
        } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
            cerr << endl
                 << "Different number of images for rgb and depth." << endl;
            return 1;
        }

        // Main loop
        cv::Mat imRGB, imD;
        for (int ni = 0; ni < nImages; ni++) {
            std::lock_guard<std::mutex> lock(slam_mutex);
            // Read image and depthmap from file
            imRGB = cv::imread(vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
            imD = cv::imread(vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);
            double tframe = vTimestamps[ni];

            if (imRGB.empty()) {
                cerr << endl
                     << "Failed to load image at: " << vstrImageFilenamesRGB[ni]
                     << endl;
                return 1;
            }

            // Pass the image to the SLAM system
            poseGrpc = SLAM->TrackRGBD(imRGB, imD, tframe);

            // Update the copy of the current map whenever a change in keyframes
            // occurs
            ORB_SLAM3::Map *currMap = SLAM->GetAtlas()->GetCurrentMap();
            std::vector<ORB_SLAM3::KeyFrame *> keyframes =
                currMap->GetAllKeyFrames();
            if (SLAM->GetTrackingState() ==
                    ORB_SLAM3::Tracking::eTrackingState::OK &&
                nkeyframes != keyframes.size()) {
                currMapPoints = currMap->GetAllMapPoints();
            }
            nkeyframes = keyframes.size();
            if (!b_continue_session) break;
        }

        cout << "Finished Processing Images\n" << endl;

        return 1;
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

    if (!b_continue_session.is_lock_free())
        return 10;  // make sure we can actually use signal
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    if (argc < 6) {
        cerr << "No args found. Expected: \n"
             << endl
             << "./bin/orb_grpc_server "
                "-data_dir=path_to_data "
                "-config_param={mode=slam_mode,} "
                "-port=grpc_port "
                "-sensors=sensor_name "
                "-data_rate_ms=frame_delay"
             << endl;
        return 1;
    }

    for (int i = 0; i < argc; ++i) {
        printf("Argument #%d is %s\n", i, argv[i]);
    }
    // setup the SLAM server
    SLAMServiceImpl slamService;
    ServerBuilder builder;

    string actual_path = argParser(argc, argv, "-data_dir=");
    if (actual_path.empty()) {
        cerr << "no data directory given" << endl;
        return 0;
    }
    string path_to_vocab = actual_path + "/config/ORBvoc.txt";
    string path_to_settings = actual_path + "/config";  // testORB.yaml";

    slamService.path_to_data =
        actual_path + "/data";  // will change in DATA 127/181

    // leaving commented for possible testing
    // string dummyPath = "/home/johnn193/slam/slam-libraries/viam-orb-slam3/";
    // slamService.path_to_data = dummyPath + "/ORB_SLAM3/officePics3";
    // slamService.path_to_sequence = "Out_file.txt";

    string config_params = argParser(argc, argv, "-config_param=");
    string slam_mode = configMapParser(config_params, "mode=");
    if (slam_mode.empty()) {
        cerr << "no SLAM mode given" << endl;
        return 0;
    }

    string slam_port = argParser(argc, argv, "-port=");
    if (slam_port.empty()) {
        cerr << "no gRPC port given" << endl;
        return 0;
    }

    string frames = argParser(argc, argv, "-data_rate_ms=");
    if (frames.empty()) {
        cerr << "No camera data rate specified" << endl;
        return 0;
    }
    slamService.frame_delay = stoi(frames);

    slamService.camera_name = argParser(argc, argv, "-sensors=");
    if (slamService.camera_name.empty()) {
        cout << "No camera given -> running in offline mode" << endl;
        slamService.offlineFlag = true;
    }

    builder.AddListeningPort(slam_port, grpc::InsecureServerCredentials());
    builder.RegisterService(&slamService);

    // Start the SLAM gRPC server
    std::unique_ptr<Server> server(builder.BuildAndStart());
    printf("Server listening on %s\n", slam_port.c_str());

    // Determine which settings file to use(.yaml)
    const path myPath(path_to_settings);
    path latest;
    std::time_t latest_tm = 0;
    for (auto &&entry :
         boost::make_iterator_range(directory_iterator(myPath), {})) {
        path p = entry.path();

        if (is_regular_file(p) && p.extension() == ".yaml") {
            std::time_t timestamp = last_write_time(p);
            cout << "this file: " << p << "\t with time:" << timestamp << endl;
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
        cerr << "no correctly formatted .yaml file found, Expected:\n"
                "{sensor}_data_{dateformat}.yaml\n";
        return 0;
    }

    // report the current yaml file check if it matches our format
    const string myYAML = latest.stem().string();
    cout << "Our yaml file: " << myYAML << endl;
    string full_path_to_settings =
        path_to_settings + "/" + latest.filename().string();
    if (slamService.offlineFlag) {
        if (myYAML.find("_data_") != string::npos)
            slamService.camera_name = myYAML.substr(0, myYAML.find("_data_"));
        else {
            cerr << "no correctly formatted .yaml file found, Expected:\n"
                    "{sensor}_data_{dateformat}.yaml\n"
                    "as most the recent config in directory\n";
            return 0;
        }
    }

    // Grab timestamp from yaml
    slamService.yamlTime = readTimeFromFilename(
        myYAML.substr(myYAML.find("_data_") + FILENAME_CONST));
    cout << "The time from our config is: " << slamService.yamlTime
         << " seconds" << endl;

    // Start SLAM
    SlamPtr SLAM = nullptr;
    boost::algorithm::to_lower(slam_mode);

    if (slam_mode == "rgbd") {
        cout << "RGBD Selected" << endl;

        // Create SLAM system. It initializes all system threads and gets ready
        // to process frames.
        SLAM = std::make_unique<ORB_SLAM3::System>(
            path_to_vocab, full_path_to_settings, ORB_SLAM3::System::RGBD,
            false, 0);
        if (slamService.offlineFlag) {
            cout << "Running in offline mode" << endl;
            slamService.process_rgbd_offline(SLAM.get());
        } else {
            cout << "Running in online mode" << endl;
            slamService.process_rgbd_online(SLAM.get());
        }
        // slamService.process_rgbd_old(SLAM);

    } else if (slam_mode == "mono") {
        // TODO implement MONO
        // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-182
    }

    SLAM->Shutdown();
    cout << "System shutdown!\n" << endl;

    return 0;
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

string configMapParser(string map, string varName) {
    string strVal;
    size_t loc = string::npos;
    stringstream ss(map);

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
    // Create a stream which we will use to parse the string,
    std::istringstream ss(filename);
    // which we provide to constructor of stream to fill the buffer.

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

void decodeBOTH(std::string filename, cv::Mat &im, cv::Mat &depth) {
    cv::Mat rawData;
    std::ifstream fin(filename + ".both");
    if (fin.peek() == std::ifstream::traits_type::eof()) {
        cerr << "Bad File, found EOF" << endl;
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

    char *pngBuf = &contents[location];

    depth = cv::Mat(cv::Size(frameWidth, frameHeight), CV_16U, depthFrame,
                    cv::Mat::AUTO_STEP);
    rawData = cv::Mat(cv::Size(1, nSize - location), CV_8UC1, (void *)pngBuf,
                      cv::IMREAD_COLOR);
    im = cv::imdecode(rawData, cv::IMREAD_COLOR);
}

int parseDataDir(const std::vector<std::string> &files,
                 FileParserMethod interest, double configTime,
                 double *timeInterest) {
    // Find the next frame based off the current interest given a directory of
    // data and time to search from
    int locInterest = -1;
    double minTime = std::numeric_limits<double>::max();
    double maxTime = 0;
    for (int i = 0; i < files.size() - 1; i++) {
        double fileTime = readTimeFromFilename(
            files[i].substr(files[i].find("_data_") + FILENAME_CONST));

        double delTime = fileTime - configTime;
        if (delTime > 0) {
            // Find the file closest to the configTime
            if (interest == Closest) {
                if (minTime > delTime) {
                    locInterest = i;
                    minTime = delTime;
                    *timeInterest = fileTime;
                }
            }
            // Find the file generated most recently
            else if (interest == Recent) {
                if (maxTime < delTime) {
                    locInterest = i;
                    maxTime = delTime;
                    *timeInterest = fileTime;
                }
            }
        }
    }
    return locInterest;
}
