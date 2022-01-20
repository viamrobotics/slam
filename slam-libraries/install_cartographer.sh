#!/bin/sh

set -o errexit
set -o verbose

rm -rf libs-cartographer

mkdir libs-cartographer
pushd libs-cartographer

popd
export LIBRARY_PATH=/usr/local/lib:/usr/local/include:
./cartographer/scripts/install_cartographer_cmake.sh
