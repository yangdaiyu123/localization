cmake_minimum_required(VERSION 2.8.3)
project(lidar_detection)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	sensor_msgs

  	pcl_conversions
  	pcl_ros
)

find_package(PCL 1.7 REQUIRED common io visualization)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include/lidar_detection
  LIBRARIES lidar_detection
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)



###########
## Build ##
###########

include_directories(
  include
  include ${catkin_INCLUDE_DIRS}
  include ${roscpp_INCLUDE_DIRS}
)

add_library(lidar_detection src/lidar_detection.cpp src/lidar_data_type.cpp include/lidar_detection/basic_operate.h)
target_link_libraries(lidar_detection ${catkin_LIBRARIES})
target_link_libraries(lidar_detection ${PCL_LIBRARIES})

add_executable(lidar_detection_test src/lib_test.cpp)
target_link_libraries(lidar_detection ${catkin_LIBRARIES})
target_link_libraries(lidar_detection ${PCL_LIBRARIES})

#############
## Install ##
#############

install(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
