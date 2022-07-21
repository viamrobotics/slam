# ORB_SLAM3

ORB_SLAM3 is a SLAM system for feature based mapping using monocular, rgbd, and stereo camera setups. 

For more information see [the official ORB_SLAM3 Repo](https://github.com/UZ-SLAMLab/ORB_SLAM3).

## Installation instructions
### Install Dependencies
Make sure to follow all steps as outline in this [here](../../README.md#setup).

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
