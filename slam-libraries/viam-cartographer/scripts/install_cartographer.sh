#!/bin/sh
set -o errexit

# Build and install Cartographer with the viam wrapper.
pushd cartographer
cp ../CMakeLists.txt .
rm -rf build
mkdir build
pushd build

# export LIBRARY_PATH=/usr/local/lib:/usr/local/include:
cmake .. -G Ninja -DCMAKE_CXX_STANDARD=17 -DCMAKE_PREFIX_PATH=`brew --prefix` -DQt5_DIR=$(brew --prefix qt5)/lib/cmake/Qt5
ninja
sudo ninja install
popd
popd

rm -rf build
mkdir build
cp cartographer/build/viam_carto_main build/main