cmake_minimum_required(VERSION 2.8.3)
project(genicam_cvb)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  camera_info_manager
  #   cv_bridge
)

find_package(OpenCV 3.1 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES genicam arghandler
 CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
# DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDES}
  "$ENV{CVB}/include"
  include
)

add_library(genicam src/genicam.cpp)

target_link_libraries(genicam
  "$ENV{CVB}/lib/genicam/libGCBase_gcc46_v3_0.so"
  "$ENV{CVB}/lib/genicam/libGenApi_gcc46_v3_0.so"
  "$ENV{CVB}/lib/libCVCDriver.so"
  "$ENV{CVB}/lib/libCVCFile.so"
  "$ENV{CVB}/lib/libCVCImg.so"
  "$ENV{CVB}/lib/libCVCUtilities.so"
  "$ENV{CVB}/lib/libCVFactory.so"
  "$ENV{CVB}/lib/libCVGenApi.so"
  "$ENV{CVB}/lib/libCVRegistry.so"
  "$ENV{CVB}/lib/libCVUAL.so"
  "$ENV{CVB}/lib/libCVBase.so"
)

add_library(arghandler src/arghandler.cpp)

add_executable(genicam_node src/genicam_node.cpp)

target_link_libraries(genicam_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  genicam
  arghandler
)
