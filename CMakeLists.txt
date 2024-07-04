cmake_minimum_required(VERSION 3.0.2)
project(pcd2laser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pluginlib
  nodelet
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcd2laser
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pcd2laser src/pcd2laser.cpp)
target_link_libraries(pcd2laser ${catkin_LIBRARIES})