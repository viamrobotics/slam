# Setup ORBSLAM3
These are the steps needed for the dependencies for ORBSLAM3. 


## setup Pangolin(includes eigen)
```
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
./scripts/install_prerequisites.sh recommended
mkdir build && cd build
cmake ..
make -j4 
sudo make install
```
## setup openCV
```
sudo apt install libopencv-dev
```
## setup Eigen3
```
sudo apt install libeigen3-dev
```

## (semi optional) Python3 - use for some post processing with ground truth data(like vicon)
```
sudo apt install libpython3.7-dev
sudo apt install python3-pip
pip3 install --upgrade pip
pip3 install numpy
pip3 install matplotlib
```
# OTHER STUFF 
```
sudo apt install libssl-dev 
sudo apt-get install libboost-all-dev
```
make changes to CMakeLists.txt if needed (change openCV version was the one I ran into)
sometimes crashes on the initial build on pis, as far as I can tell during the sophus build. 
in build.sh you may want to tweak the make -j flags 

# Using ORBSLAM3 
After setting up the dependencies, build orbslam using 
```
cd viam-orb-slam3
./build_orbslam.sh
```
This will build your binary orb_grpc_server in `viam-orb-slam3\bin`. Move this binary into usr/local/bin
```
sudo cp bin/orb_grpc_server /usr/local/bin/
```
In your desired data directory, move the vocabulary file from orbslam into your `~/config` folder. You only have to do this once per data directory. 
```
sudo cp ORB_SLAM3/Vocabulary/ORBvoc.txt ~/YOUR_DATA_DIR/config/
```