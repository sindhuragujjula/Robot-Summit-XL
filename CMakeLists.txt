cmake_minimum_required(VERSION 2.8.3)
project(new_hardware_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

## Declare ROS messages and services

## Generate added messages and services

## Declare a catkin package
catkin_package()

## Build detection and analyse_hardware
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(detection src/detection.cpp)
target_link_libraries(detection ${catkin_LIBRARIES})

add_executable(analyse_hardware src/analyse_hardware.cpp)
target_link_libraries(analyse_hardware ${catkin_LIBRARIES} ${LIBUSB1_LIBRARIES})

add_definitions(-std=c++0x)
