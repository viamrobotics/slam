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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "proto/api/common/v1/common.grpc.pb.h"
#include "proto/api/common/v1/common.pb.h"
#include "proto/api/service/slam/v1/slam.grpc.pb.h"
#include "proto/api/service/slam/v1/slam.pb.h"
#define _USE_MATH_DEFINES
using namespace std;

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

bool b_continue_session;

void exit_loop_handler(int s) {
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

void LoadImagesRGBD(const string &pathSeq, const string &strPathTimes,
                    vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD,
                    vector<double> &vTimeStamps);
void SavePCD(std::vector<ORB_SLAM3::MapPoint *> mapStuff, string file_name);

class SLAMServiceImpl final : public SLAMService::Service {
   public:
    ::grpc::Status GetPosition(ServerContext *context,
                               const GetPositionRequest *request,
                               GetPositionResponse *response) override {
        // Copy pose to new location
        Sophus::SE3f currPose(poseGrpc);

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
            // is wrt the camera(z is coming out of the lens) so may need to
            // transform.
            std::vector<ORB_SLAM3::MapPoint *> actualMap(currMapPoints);

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

            // write the map with simple rgb colors based off height from the
            // "ground". Map written as a binary
            for (auto p : actualMap) {
                Eigen::Matrix<float, 3, 1> v = p->GetWorldPos();
                float val = v.y();
                auto ratio = (val - min) / span;
                clr = (char)(60 + (ratio * 192));
                if (clr > 255) clr = 255;
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
        }
        return grpc::Status::OK;
    }

    int process_rgbd() {
        // Function used for ORB_SLAM with an rgbd camera. Currently returns int
        // as a placeholder for error signals should the server have to restart
        // itself.
        string file_nameTraj = output_file_name + ".txt";
        string file_nameKey = output_file_name + "Keyframe.txt";
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

        // Create SLAM system. It initializes all system threads and gets ready
        // to process frames.
        ORB_SLAM3::System SLAM(path_to_vocab, path_to_settings,
                               ORB_SLAM3::System::RGBD, false, 0,
                               file_nameTraj);

        // Main loop
        cv::Mat imRGB, imD;
        for (int ni = 0; ni < nImages; ni++) {
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
            poseGrpc = SLAM.TrackRGBD(imRGB, imD, tframe);

            // Update the copy of the current map whenever a change in keyframes
            // occurs
            ORB_SLAM3::Map *currMap = SLAM.GetAtlas()->GetCurrentMap();
            std::vector<ORB_SLAM3::KeyFrame *> keyframes =
                currMap->GetAllKeyFrames();
            if (SLAM.GetTrackingState() ==
                    ORB_SLAM3::Tracking::eTrackingState::OK &&
                nkeyframes != keyframes.size()) {
                currMapPoints = currMap->GetAllMapPoints();
            }
            nkeyframes = keyframes.size();
        }

        cout << "System shutdown!\n" << endl;
        std::vector<ORB_SLAM3::MapPoint *> mapStuff =
            SLAM.GetAtlas()->GetCurrentMap()->GetAllMapPoints();
        SavePCD(mapStuff, output_file_name);
        SLAM.Shutdown();
        return 1;
    }

    Sophus::SE3f poseGrpc;
    std::vector<ORB_SLAM3::MapPoint *> currMapPoints;
    string output_file_name;
    string slam_port;
    string path_to_vocab;
    string path_to_settings;
    string path_to_data;
    string path_to_sequence;
};

int main(int argc, char **argv) {
    // TODO: change inputs to match args from rdk
    // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-179
    if (argc < 5) {
        cerr << endl
             << "Usage: binary path_to_vocabulary path_to_settings "
                "path_to_sequence_folder_1 path_to_times_file_1 "
                "(trajectory_file_name)"
             << endl;
        cerr << endl
             << "./bin/viam_main_v1 "
                "./ORB_SLAM3/Vocabulary/ORBvoc.txt "
                "./ORB_SLAM3/initialAttempt/"
                "realsense515_depth2.yaml "
                "./ORB_SLAM3/officePics3 Out_file.txt "
                "outputPose"
             << endl;
        return 1;
    }

    // setup the SLAM server
    SLAMServiceImpl slamService;
    ServerBuilder builder;

    slamService.path_to_vocab = string(argv[1]);
    slamService.path_to_settings = string(argv[2]);
    slamService.path_to_data = string(argv[3]);
    slamService.path_to_sequence = string(argv[4]);
    slamService.output_file_name = string(argv[5]);
    string slam_mode = "RGBD";
    string port_num = "8085";
    slamService.slam_port = "localhost:" + port_num;

    builder.AddListeningPort(slamService.slam_port,
                             grpc::InsecureServerCredentials());
    builder.RegisterService(&slamService);

    // Start the SLAM gRPC server
    std::unique_ptr<Server> server(builder.BuildAndStart());
    printf("Server listening on %s", slamService.slam_port);

    if (slam_mode == "RGBD") {
        // TODO update to work with images from rdk
        // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-181
        cout << "RGBD SELECTED" << endl;
        slamService.process_rgbd();

    } else if (slam_mode == "MONO") {
        // TODO implement MONO
        // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-182
    }

    return 0;
}

void LoadImagesRGBD(const string &pathSeq, const string &strPathTimes,
                    vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD,
                    vector<double> &vTimeStamps) {
    string pathCam0 = pathSeq + "/rgb";
    string pathCam1 = pathSeq + "/depth";
    ifstream fTimes;
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

void SavePCD(std::vector<ORB_SLAM3::MapPoint *> mapStuff, string file_name) {
    string pathSaveFileName = "./";
    pathSaveFileName = pathSaveFileName.append(file_name);
    pathSaveFileName.append(".pcd");
    std::remove(pathSaveFileName.c_str());
    FILE *fp = fopen(pathSaveFileName.c_str(), "w");
    fprintf(fp,
            "VERSION .7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH %li\n"
            "HEIGHT %i\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS %li\nDATA ascii\n",
            mapStuff.size(), 1, mapStuff.size());
    for (auto p : mapStuff) {
        Eigen::Matrix<float, 3, 1> v = p->GetWorldPos();
        fprintf(fp, "%f %f %f\n", v.x(), v.y(), v.z());
        // ofs << v.x() << " " << v.y() << " " << v.z() << "\n";
    }
    fclose(fp);
}
