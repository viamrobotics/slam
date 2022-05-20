cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM_CUSTOM)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(RDK_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../../rdk)
set(SRCDIR ${RDK_SOURCE_DIR}/grpc/cpp/gen)
set(ORBSLAM_SOURCE_DIR ${PROJECT_SOURCE_DIR}/ORB_SLAM3)
message("PROJECT SOURCE DIR: " ${PROJECT_SOURCE_DIR})
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

find_package(OpenCV 4.5.1)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 3.2 not found.")
endif()

message("OPENCV VERSION:")
message(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${ORBSLAM_SOURCE_DIR}
  ${ORBSLAM_SOURCE_DIR}/include
  ${ORBSLAM_SOURCE_DIR}/include/CameraModels
  ${ORBSLAM_SOURCE_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_subdirectory(${ORBSLAM_SOURCE_DIR}/Thirdparty/g2o)

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
  -lcrypto)
