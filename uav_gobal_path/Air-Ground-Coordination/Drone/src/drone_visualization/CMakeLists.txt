cmake_minimum_required(VERSION 2.8.3)
project(drone_visualization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")
ADD_COMPILE_OPTIONS(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib nav_msgs geometry_msgs std_msgs visualization_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(drone_visualization src/drone_visualization.cpp)

target_link_libraries(drone_visualization
   ${catkin_LIBRARIES}
)

install(TARGETS drone_visualization
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
