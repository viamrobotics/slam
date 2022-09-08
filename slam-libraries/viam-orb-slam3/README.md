# ORB_SLAM3

ORB_SLAM3 is a SLAM system for feature based mapping using monocular, rgbd, and stereo camera setups. 

For more information see [the official ORB_SLAM3 Repo](https://github.com/UZ-SLAMLab/ORB_SLAM3).

## Installation instructions
Make sure to follow all steps as outlined in [the setup section here](../../README.md#setup) in addition to the steps below. 

### Automatic Dependency Installation
To automatically install dependencies, use the target 
```
./setup_orbslam.sh
```

which installs all dependencies required for ORB_SLAM3. the dependencies installed this way are
```
cmake libglew-dev libopencv-dev libeigen3-dev libssl-dev libboost-all-dev libpangolin-dev
```
### Manual Dependency Install
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
# Other dependencies
sudo apt install libssl-dev 
sudo apt-get install libboost-all-dev
```

### Build ORB_SLAM3
ensure gRPC is setup within [slam-libraries](../.) using 
```
make pull-rdk
```

To build ORB_SLAM3 run the following
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
You only have to do this once per data directory. Note ORB_SLAM3 will fail if the Vocabulary cannot be found
