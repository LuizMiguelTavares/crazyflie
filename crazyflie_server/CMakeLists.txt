cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

find_package(crazyflie_cpp REQUIRED)

# 1) Find libusb-1.0 using pkg-config
find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB REQUIRED libusb-1.0)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/crazyflie_cpp
  ${LIBUSB_INCLUDE_DIRS}  # Make sure libusb headers are included
)

add_executable(crazyflie_server src/crazyflie_server.cpp)
target_link_libraries(crazyflie_server
  ${catkin_LIBRARIES}
  crazyflie_cpp
  ${LIBUSB_LIBRARIES}
)