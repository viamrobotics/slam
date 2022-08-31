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
set(Pangolin_DIR ${PROJECT_SOURCE_DIR}/Pangolin/build)

find_package(Pangolin REQUIRED)
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${ORBSLAM_SOURCE_DIR}
  ${ORBSLAM_SOURCE_DIR}/include
  ${ORBSLAM_SOURCE_DIR}/include/CameraModels
  ${ORBSLAM_SOURCE_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS})
  
# Build examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(viam_main_v1 main_v1.cc)
target_link_libraries(
  viam_main_v1
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORBSLAM_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORBSLAM_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${ORBSLAM_SOURCE_DIR}/lib/libORB_SLAM3.so
  -lboost_serialization
  -lboost_filesystem
  -lcrypto)

if(EXISTS ${SRCDIR})
  find_package(PkgConfig REQUIRED)

  pkg_check_modules(GRPC REQUIRED grpc)
  pkg_check_modules(GRPCPP REQUIRED grpc++)
  pkg_check_modules(PROTOBUF REQUIRED protobuf)

  set(SOURCES ${SRCDIR}/proto/api/service/slam/v1/slam.grpc.pb.cc
              ${SRCDIR}/proto/api/service/slam/v1/slam.pb.cc)
  set(SOURCES ${SOURCES} ${SRCDIR}/proto/api/common/v1/common.grpc.pb.cc
              ${SRCDIR}/proto/api/common/v1/common.pb.cc)
  set(SOURCES ${SOURCES} ${SRCDIR}/google/api/annotations.pb.cc
              ${SRCDIR}/google/api/httpbody.pb.cc)
  set(SOURCES ${SOURCES} ${SRCDIR}/google/api/http.pb.cc)

  add_executable(orb_grpc_server orbslam_server_v1.cc ${SOURCES})
  target_link_libraries(
    orb_grpc_server
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${ORBSLAM_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORBSLAM_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${ORBSLAM_SOURCE_DIR}/lib/libORB_SLAM3.so
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
