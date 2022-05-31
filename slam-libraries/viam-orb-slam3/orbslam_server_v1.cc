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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>

#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include "proto/api/common/v1/common.grpc.pb.h"
#include "proto/api/common/v1/common.pb.h"
#include "proto/api/service/slam/v1/slam.grpc.pb.h"
#include "proto/api/service/slam/v1/slam.pb.h"

using namespace std;

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;

using proto::api::service::slam::v1::SLAMService;
using proto::api::service::slam::v1::GetPositionRequest;
using proto::api::service::slam::v1::GetPositionResponse;
using proto::api::service::slam::v1::GetMapRequest;
using proto::api::service::slam::v1::GetMapResponse;

class SLAMServiceImpl final : public SLAMService::Service {
    public:
    ::grpc::Status GetPosition(ServerContext* context,
                               const GetPositionRequest* request,
                               GetPositionResponse* response) override {
        
        return grpc::Status::OK;
    }
    ::grpc::Status GetMap(ServerContext* context,
                          const GetMapRequest* request,
                          GetMapResponse* response) override {
        
        return grpc::Status::OK;
    }
};
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
             << "./viam-orb-slam3/bin/viam_main_v1 "
                "./viam-orb-slam3/ORB_SLAM3/Vocabulary/ORBvoc.txt "
                "./viam-orb-slam3/ORB_SLAM3/initialAttempt/"
                "realsense515_depth2.yaml "
                "./viam-orb-slam3/ORB_SLAM3/officePics3 Out_file.txt "
                "outputPose"
             << endl;
        return 1;
    }
    string path_to_vocab = string(argv[1]);
    string path_to_settings = string(argv[2]);
    string path_to_data = string(argv[3]);
    string path_to_sequence = string(argv[4]);
    string output_file_name = string(argv[5]);
    string file_name, file_nameTraj, file_nameKey;
    string slam_mode = "RGBD";
    string port_num = "8085";
    string slam_port = "localhost:" +port_num;

    SLAMServiceImpl slamService;
    ServerBuilder builder;
    builder.AddListeningPort(slam_port,
                             grpc::InsecureServerCredentials());
    builder.RegisterService(&slamService);
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Server listening on "
                << slam_port << std::endl;


    file_name = output_file_name;
    file_nameTraj = file_name;
    file_nameKey = file_name;
    file_nameTraj = file_nameTraj.append(".txt");
    file_nameKey = file_nameKey.append("Keyframe.txt");
    int nImages = 0;
    if (slam_mode == "RGBD") {
        // TODO update to work with images from rdk
        // https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-181
        cout << "RGBD SELECTED " << endl;
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
        float imageScale = SLAM.GetImageScale();

        // Main loop
        cv::Mat imRGB, imD;
        Sophus::SE3f pose;

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
            pose = SLAM.TrackRGBD(imRGB, imD, tframe);
        }
        cout << "System shutdown!\n";
        std::vector<ORB_SLAM3::MapPoint*> mapStuff = SLAM.GetAtlas()->GetCurrentMap()->GetAllMapPoints();
        SavePCD(mapStuff, file_name);
        SLAM.Shutdown();
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
    std::ofstream ofs(pathSaveFileName, std::ios::binary);
    ofs << "VERSION .7\n"
        << "FIELDS x y z\n"
        << "SIZE 4 4 4\n"
        << "TYPE F F F\n"
        << "COUNT 1 1 1\n"
        << "WIDTH " << mapStuff.size() << "\n"
        << "HEIGHT " << 1 << "\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\n"
        << "POINTS " << mapStuff.size() << "\n"
        << "DATA ascii\n";
    for (auto p : mapStuff) {
        Eigen::Matrix<float, 3, 1> v = p->GetWorldPos();
        ofs << v.x() << " " << v.y() << " " << v.z() << "\n";
    }
    ofs.close();
}
