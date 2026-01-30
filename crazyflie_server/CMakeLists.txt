cmake_minimum_required(VERSION 3.0.2)
project(crazyflie_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

# Submodule (inclui crazyflie-link-cpp por dentro)
add_subdirectory(deps/crazyflie_cpp)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
  LIBRARIES crazyflie_cpp crazyflieLinkCpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/deps/crazyflie_cpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/deps/crazyflie_cpp/crazyflie-link-cpp/include
)

#add_executable(crazyflie_server src/crazyflie_server.cpp)

# AQUI é o ponto: linkar as duas libs
#target_link_libraries(crazyflie_server
#  ${catkin_LIBRARIES}
#  crazyflie_cpp
#  crazyflieLinkCpp
#)

# Só pra debug (opcional)
message(STATUS "Targets: crazyflie_cpp=$<TARGET_EXISTS:crazyflie_cpp> crazyflieLinkCpp=$<TARGET_EXISTS:crazyflieLinkCpp>")
