#!/bin/sh
set -o errexit

BASEDIR=`pwd`

cd $BASEDIR/ORB_SLAM3
echo "Configuring and building ORB_SLAM3 ..."
cd build
make -j`nproc`

cd $BASEDIR
cd build
make -j`nproc`
