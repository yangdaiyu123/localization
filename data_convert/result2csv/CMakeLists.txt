cmake_minimum_required(VERSION 2.8.3)
project(result2csv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_follower
  CATKIN_DEPENDS roscpp std_msgs tf
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

add_executable(result2csv src/result2csv.cpp)
target_link_libraries(result2csv ${catkin_LIBRARIES})



