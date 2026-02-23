cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_copilot)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crazyflie_copilot
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(crazyflie_copilot src/crazyflie_copilot_smc.cpp)

target_link_libraries(crazyflie_copilot
  ${catkin_LIBRARIES}
)