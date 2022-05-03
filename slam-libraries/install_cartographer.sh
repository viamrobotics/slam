#!/bin/sh
set -o errexit

# export LIBRARY_PATH=/usr/local/lib:/usr/local/include:
./cartographer/viam/scripts/install_cartographer_cmake.sh

rm -rf cartographer/viam/build
mkdir cartographer/viam/build

cp ./cartographer/build/viam_carto_main ./cartographer/viam/build/main
