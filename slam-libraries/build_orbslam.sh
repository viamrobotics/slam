cd ORB_SLAM_CUSTOM
BASEDIR=`pwd`
cd $BASEDIR/ORB_SLAM3/Thirdparty
ORB_THIRDPARTYDIR=`pwd`

echo "Configuring and building Thirdparty/DBoW2 ..."
#ORBSLAM used for place recognition
cd $ORB_THIRDPARTYDIR/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`

echo "Configuring and building Thirdparty/g2o ..."
#ORBSLAM used for nonlinear optimization
cd $ORB_THIRDPARTYDIR/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`

echo "Configuring and building Thirdparty/Sophus ..."
#ORBSLAM used for lie groups with 2D and 3D geometric problems
cd $ORB_THIRDPARTYDIR/Sophus
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`

cd $BASEDIR/ORB_SLAM3

echo "Uncompress vocabulary ..."

tar -xf ORBvoc.txt.tar.gz -C Vocabulary/

echo "Configuring and building ORB_SLAM3 ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`

cd $BASEDIR
mkdir bin
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`
