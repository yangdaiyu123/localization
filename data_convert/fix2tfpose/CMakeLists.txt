cmake_minimum_required(VERSION 2.8.3)
project(fix2pose)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  gps
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_follower
  CATKIN_DEPENDS roscpp std_msgs tf gps
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fix2tfpose src/fix2tfpose.cpp)
target_link_libraries(fix2tfpose ${catkin_LIBRARIES})
target_link_libraries(fix2tfpose gps)
