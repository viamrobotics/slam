sudo apt install libopencv-dev libeigen3-dev libssl-dev libboost-all-dev

git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
./scripts/install_prerequisites.sh recommended
mkdir build
cd build
echo `pwd`
cmake ..
make -j4 
sudo make install
