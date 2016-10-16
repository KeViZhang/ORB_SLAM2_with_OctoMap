cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#rosbuild_init()

#IF(NOT ROS_BUILD_TYPE)
#  SET(ROS_BUILD_TYPE Release)
#ENDIF()

#MESSAGE("Build type: " ${ROS_BUILD_TYPE})


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 2.4.10 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(QGLViewer)
#find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost 1.54.0 REQUIRED COMPONENTS system filesystem)
#find_package(Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS})
link_directories( ${Boost_LIBRARY_DIRS})
#target_link_libraries( ${Boost_LIBRARIES})
FIND_PACKAGE(octomap REQUIRED )
#FIND_PACKAGE(octovis REQUIRED )
INCLUDE_DIRECTORIES(${OCTOMAP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${QGLVIEWER_INCLUDE_DIRS})
#TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${OCTOMAP_LIBRARIES})

#find_package(PCL 1.7 REQUIRED COMPONENTS common io)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/pointcloudmapping.cc
#src/point_cloud_compression.cpp
#src/test.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#${PCL_INCLUDE_DIRS}
${Boost_LIBRARIES}
#${PCL_LIBRARIES}
${OCTOMAP_LIBRARIES}
${QGLViewer_LIBRARIES}
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_lhc
Examples/RGB-D/rgbd_lhc.cc)
target_link_libraries(rgbd_lhc ${PROJECT_NAME} ${PCL_LIBRARIES})

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME} ${PCL_LIBRARIES})

#add_executable(rgbd_lhc
#Examples/RGB-D/rgbd_lhc.cc)
#target_link_libraries(rgbd_lhc ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME} ${PCL_LIBRARIES})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME} ${PCL_LIBRARIES})

#add_executable(mono_kitti
#Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})


