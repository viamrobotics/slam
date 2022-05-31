#!/bin/sh
set -o errexit

# Build and install Cartographer with the viam wrapper.
pushd cartographer
cp ../cartographer_build_utils/CMakeLists.txt CMakeLists.txt
rm -rf build
mkdir build
pushd build

cmake .. -G Ninja -DCMAKE_CXX_STANDARD=17 -DCMAKE_PREFIX_PATH=`brew --prefix`
ninja
sudo ninja install
popd
popd
