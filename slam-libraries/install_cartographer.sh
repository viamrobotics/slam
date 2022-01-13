#!/bin/sh
mkdir libs-cartographer
pushd libs-cartographer


# Make sure: Running tests works or not?
./../cartographer/scripts/install_abseil.sh
./../cartographer/scripts/install_ceres.sh
./../cartographer/scripts/install_proto3.sh

popd
./cartographer/scripts/install_cartographer_cmake.sh
