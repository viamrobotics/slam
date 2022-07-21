# ORB_SLAM3

ORB_SLAM3 is a SLAM system for feature based mapping using monocular, rgbd, and stereo camera setups. 

For more information see [the official ORB_SLAM3 Repo](https://github.com/UZ-SLAMLab/ORB_SLAM3).

## Installation instructions

### Install Dependencies
Make sure to follow all steps as outlined in [the setup section here](../../README.md#setup).

```bash
# Install & build Pangolin (includes eigen)
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
./scripts/install_prerequisites.sh recommended
mkdir build && cd build
cmake ..
make -j4 
sudo make install
```

```bash
# Install openCV
sudo apt install libopencv-dev
```

```bash
# Install Eigen3
sudo apt install libeigen3-dev
```

```bash
# [advanced] Install Python3
# This is only needed for visualizing trajectories when using built-in ORB_SLAM3 functions.
# It is used for post-processing with ground truth data (like vicon).
sudo apt install libpython3.7-dev
sudo apt install python3-pip
pip3 install --upgrade pip
pip3 install numpy
pip3 install matplotlib
```

```bash
# Other dependencies
sudo apt install libssl-dev 
sudo apt-get install libboost-all-dev
```

### Build ORB_SLAM3

```bash
cd viam-orb-slam3
./build_orbslam.sh
```

This will build the binary and save it at `./bin/orb_grpc_server`. Move this binary into `usr/local/bin` by running:

```bash
sudo cp bin/orb_grpc_server /usr/local/bin/
```

In your desired data directory, move the vocabulary file from orbslam into your `~/config` folder:  
```bash
sudo cp ORB_SLAM3/Vocabulary/ORBvoc.txt ~/YOUR_DATA_DIR/config/
```
You only have to do this once per data directory.

### NOTE

The (initial) build can crash on RPIs, which seems to happen during the sophus build. In this case, it might be helpful change the `make -j` flags in [build_orbslam.sh](./build_orbslam.sh) from 

```bash
make -j`nproc`
```

to

```bash
make -j2
```

Besides that, it might happen that other changes are needed to be made in [CMakeLists.txt](./CMakeLists.txt). In one case, the OpenCV version had to be changed in order for ORB_SLAM3 to compile.

## Running ORB_SLAM3
You can run the binary `./bin/orb_grpc_server` directly.

### Local Development with saved data
The following describes how one can set up a dev environment using RDK and locally saved data on an RPI.

In [orbslam_server_v1.cc](./orbslam_server_v1.cc), find the following lines in the `main` function:

```cpp
    // leaving commented for possible testing
    // string dummyPath = "/home/johnn193/slam/slam-libraries/viam-orb-slam3/";
    // slamService.path_to_data = dummyPath + "/ORB_SLAM3/officePics3";
    // slamService.path_to_sequence = "Out_file.txt";
```

and uncomment them. Change the paths to point to the directory that contains your data.

Also, find this part of the code in the `main` function:

```cpp
    if (slam_mode == "rgbd") {
        BOOST_LOG_TRIVIAL(info) << "RGBD selected";

        // Create SLAM system. It initializes all system threads and gets ready
        // to process frames.
        SLAM = std::make_unique<ORB_SLAM3::System>(
            path_to_vocab, full_path_to_settings, ORB_SLAM3::System::RGBD,
            false, 0);
        if (slamService.offlineFlag) {
            BOOST_LOG_TRIVIAL(info) << "Running in offline mode";
            slamService.process_rgbd_offline(SLAM.get());
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
        ...
    }
```

and make sure to copy/paste the `slamService.process_rgbd_old(SLAM);` line into this `if` statement:

```cpp
    if (slamService.offlineFlag) {
        BOOST_LOG_TRIVIAL(info) << "Running in offline mode";
        // slamService.process_rgbd_offline(SLAM.get());
        slamService.process_rgbd_old(SLAM);
        ...
```


Your data folder should have the following structure:

```
YOUR_PATH_TO_DATA
 |
 |- config
 |  |- YOUR.yaml
 |  |- ORBvoc.txt
 |
 |- YOUR_DATA
    |- depth/...
    |- rgb/...
    |- sampleDataset.osa
    |- sampleDatasetTXT.osa
```

After that, build the code and copy/paste the binary file as described above. Run RDK with an appropriate `MY_CONFIG.json` file: `go run web/cmd/server/main.go MY_CONFIG.json `.

Every time you make an edit, you have to build and copy/paste the binary file as described above. 

Alternatively, you can edit code in RDK to point to the binary that's saved at `bin/orb_grpc_server` by editing `BinaryLocation` in `services/slam/slamlibraries.go` like this:

```go
var orbslamv3Metadata = LibraryMetadata{
    AlgoName:       "orbslamv3",
    AlgoType:       sparse,
    SlamMode:       map[string]mode{"mono": mono, "rgbd": rgbd},
    BinaryLocation: "YOUR_PATH_TO_SLAM_LIBRARY/slam/slam-libraries/viam-orb-slam3/bin/orb_grpc_server",
}
```
This way, you won't have to copy the binary to `/usr/local/bin` everytime you build orbslam.

If you have questions or need files/data, ask John, Tess, Jeremy, or Kat.