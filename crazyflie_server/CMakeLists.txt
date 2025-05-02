cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_server)

# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

# Add crazyflie_cpp (which itself adds crazyflie-link-cpp)
add_subdirectory(deps/crazyflie_cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/deps/crazyflie_cpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/deps/crazyflie_cpp/crazyflie-link-cpp/include
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
  LIBRARIES crazyflie_cpp
)

add_executable(crazyflie_server src/crazyflie_server.cpp)
target_link_libraries(crazyflie_server
  ${catkin_LIBRARIES}
  crazyflie_cpp
)

get_target_property(cflib_exists crazyflieLinkCpp TYPE)
message(STATUS "crazyflieLinkCpp target exists and is a: ${cflib_exists}")