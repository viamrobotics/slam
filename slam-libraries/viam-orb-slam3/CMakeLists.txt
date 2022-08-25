cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM_CUSTOM)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -DBOOST_LOG_DYN_LINK")

if(DEFINED ENV{RDK_SOURCE_DIR})
  set(RDK_SOURCE_DIR $ENV{RDK_SOURCE_DIR})
else()
  set(RDK_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../rdk-minimal)
endif()

set(SRCDIR ${RDK_SOURCE_DIR}/grpc/cpp/gen)
set(ORBSLAM_SOURCE_DIR ${PROJECT_SOURCE_DIR}/ORB_SLAM3)
message("PROJECT SOURCE DIR: " ${PROJECT_SOURCE_DIR})
message("RDK SOURCE DIR: " ${RDK_SOURCE_DIR})
message("GRPC SOURCE DIR: " ${SRCDIR})

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC14)
  message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(
    FATAL_ERROR
      "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler."
  )
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.2)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.2 not found.")
  endif()
endif()

message("OPENCV VERSION:")
message(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${ORBSLAM_SOURCE_DIR}
  ${ORBSLAM_SOURCE_DIR}/src
  ${ORBSLAM_SOURCE_DIR}/include
  ${ORBSLAM_SOURCE_DIR}/include/CameraModels
  ${ORBSLAM_SOURCE_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS})

set(SOURCES ${ORBSLAM_SOURCE_DIR}/src/System.cc
            ${ORBSLAM_SOURCE_DIR}/src/Tracking.cc
            ${ORBSLAM_SOURCE_DIR}/src/LocalMapping.cc
            ${ORBSLAM_SOURCE_DIR}/src/LoopClosing.cc
            ${ORBSLAM_SOURCE_DIR}/src/ORBextractor.cc
            ${ORBSLAM_SOURCE_DIR}/src/ORBmatcher.cc
            ${ORBSLAM_SOURCE_DIR}/src/FrameDrawer.cc
            ${ORBSLAM_SOURCE_DIR}/src/Converter.cc
            ${ORBSLAM_SOURCE_DIR}/src/MapPoint.cc
            ${ORBSLAM_SOURCE_DIR}/src/KeyFrame.cc
            ${ORBSLAM_SOURCE_DIR}/src/Atlas.cc
            ${ORBSLAM_SOURCE_DIR}/src/Map.cc
            ${ORBSLAM_SOURCE_DIR}/src/MapDrawer.cc
            ${ORBSLAM_SOURCE_DIR}/src/Optimizer.cc
            ${ORBSLAM_SOURCE_DIR}/src/Frame.cc
            ${ORBSLAM_SOURCE_DIR}/src/KeyFrameDatabase.cc
            ${ORBSLAM_SOURCE_DIR}/src/Sim3Solver.cc
            ${ORBSLAM_SOURCE_DIR}/src/Viewer.cc
            ${ORBSLAM_SOURCE_DIR}/src/ImuTypes.cc
            ${ORBSLAM_SOURCE_DIR}/src/G2oTypes.cc
            ${ORBSLAM_SOURCE_DIR}/src/CameraModels/Pinhole.cpp
            ${ORBSLAM_SOURCE_DIR}/src/CameraModels/KannalaBrandt8.cpp
            ${ORBSLAM_SOURCE_DIR}/src/OptimizableTypes.cpp
            ${ORBSLAM_SOURCE_DIR}/src/MLPnPsolver.cpp
            ${ORBSLAM_SOURCE_DIR}/src/GeometricTools.cc
            ${ORBSLAM_SOURCE_DIR}/src/TwoViewReconstruction.cc
            ${ORBSLAM_SOURCE_DIR}/src/Config.cc
            ${ORBSLAM_SOURCE_DIR}/src/Settings.cc
            ${ORBSLAM_SOURCE_DIR}/include/System.h
            ${ORBSLAM_SOURCE_DIR}/include/Tracking.h
            ${ORBSLAM_SOURCE_DIR}/include/LocalMapping.h
            ${ORBSLAM_SOURCE_DIR}/include/LoopClosing.h
            ${ORBSLAM_SOURCE_DIR}/include/ORBextractor.h
            ${ORBSLAM_SOURCE_DIR}/include/ORBmatcher.h
            ${ORBSLAM_SOURCE_DIR}/include/FrameDrawer.h
            ${ORBSLAM_SOURCE_DIR}/include/Converter.h
            ${ORBSLAM_SOURCE_DIR}/include/MapPoint.h
            ${ORBSLAM_SOURCE_DIR}/include/KeyFrame.h
            ${ORBSLAM_SOURCE_DIR}/include/Atlas.h
            ${ORBSLAM_SOURCE_DIR}/include/Map.h
            ${ORBSLAM_SOURCE_DIR}/include/MapDrawer.h
            ${ORBSLAM_SOURCE_DIR}/include/Optimizer.h
            ${ORBSLAM_SOURCE_DIR}/include/Frame.h
            ${ORBSLAM_SOURCE_DIR}/include/KeyFrameDatabase.h
            ${ORBSLAM_SOURCE_DIR}/include/Sim3Solver.h
            ${ORBSLAM_SOURCE_DIR}/include/Viewer.h
            ${ORBSLAM_SOURCE_DIR}/include/ImuTypes.h
            ${ORBSLAM_SOURCE_DIR}/include/G2oTypes.h
            ${ORBSLAM_SOURCE_DIR}/include/CameraModels/GeometricCamera.h
            ${ORBSLAM_SOURCE_DIR}/include/CameraModels/Pinhole.h
            ${ORBSLAM_SOURCE_DIR}/include/CameraModels/KannalaBrandt8.h
            ${ORBSLAM_SOURCE_DIR}/include/OptimizableTypes.h
            ${ORBSLAM_SOURCE_DIR}/include/MLPnPsolver.h
            ${ORBSLAM_SOURCE_DIR}/include/GeometricTools.h
            ${ORBSLAM_SOURCE_DIR}/include/TwoViewReconstruction.h
            ${ORBSLAM_SOURCE_DIR}/include/SerializationUtils.h
            ${ORBSLAM_SOURCE_DIR}/include/Config.h
            ${ORBSLAM_SOURCE_DIR}/include/Settings.h)

add_subdirectory(${ORBSLAM_SOURCE_DIR}/Thirdparty/g2o)

# Build examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(viam_main_v1 main_v1.cc ${SOURCES})
target_link_libraries(viam_main_v1 
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORBSLAM_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORBSLAM_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
-lboost_filesystem
-lboost_log
-lcrypto)



if(EXISTS ${SRCDIR})
  find_package(PkgConfig REQUIRED)

  pkg_check_modules(GRPC REQUIRED grpc)
  pkg_check_modules(GRPCPP REQUIRED grpc++)
  pkg_check_modules(PROTOBUF REQUIRED protobuf)

  set(SOURCES ${SOURCES} ${SRCDIR}/proto/api/service/slam/v1/slam.grpc.pb.cc
              ${SRCDIR}/proto/api/service/slam/v1/slam.pb.cc)
  set(SOURCES ${SOURCES} ${SRCDIR}/proto/api/common/v1/common.grpc.pb.cc
              ${SRCDIR}/proto/api/common/v1/common.pb.cc)
  set(SOURCES ${SOURCES} ${SRCDIR}/google/api/annotations.pb.cc
              ${SRCDIR}/google/api/httpbody.pb.cc)
  set(SOURCES ${SOURCES} ${SRCDIR}/google/api/http.pb.cc)

  add_executable(orb_grpc_server orbslam_server_v1.cc ${SOURCES})
  target_link_libraries(orb_grpc_server 
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORBSLAM_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORBSLAM_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  -lboost_serialization
  -lboost_filesystem
  -lboost_log
  -lcrypto
    ${GRPC_LIBRARIES}
    ${GRPCPP_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
  target_include_directories(
    orb_grpc_server PUBLIC ${SRCDIR} ${GRPC_INCLUDE_DIRS} ${GRPCPP_INCLUDE_DIRS}
                       ${PROTOBUF_INCLUDE_DIRS})
  target_compile_options(
    orb_grpc_server PUBLIC ${GRPC_CFLAGS_OTHER} ${GRPCPP_CFLAGS_OTHER}
                       ${PROTOBUF_CFLAGS_OTHER})

else()
  message(
    FATAL_ERROR
      "RDK C++ gRPC gen files not found. Please specify the absolute path to your RDK_SOURCE_DIR e.g. 'export RDK_SOURCE_DIR=/home/user/rdk'"
  )
endif()
