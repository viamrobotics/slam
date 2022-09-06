echo "Installing ORB_SLAM3 external dependencies"
sudo apt install libopencv-dev libeigen3-dev libssl-dev libboost-all-dev
echo "Configuring and building Pangolin..."
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
./scripts/install_prerequisites.sh recommended
mkdir build
cd build
cmake ..
make -j4 
