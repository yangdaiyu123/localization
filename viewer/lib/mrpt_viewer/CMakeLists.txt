cmake_minimum_required(VERSION 2.8.3)
project(mrpt_viewer)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	sensor_msgs

  	pcl_conversions
  	pcl_ros

  	mrpt_bridge
	mrpt_map
	mrpt_msgs
	mrpt_local_obstacles
	mrpt_localization
)

find_package(PCL 1.7 REQUIRED common io visualization)
find_package(MRPT REQUIRED base slam gui opengl maps obs nav)

#add_message_files(
#  	FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

#generate_dynamic_reconfigure_options(

#)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mrpt_viewer
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
  sensor_msgs
)

include_directories(
	include ${catkin_INCLUDE_DIRS}
	include ${roscpp_INCLUDE_DIRS}
	include/${PROJECT_NAME}/
)

# Set the name to use for the executable.

## Set the source files to use with the executable.

# Build the executable that will be used to run this node.

add_library(mrpt_viewer src/mrpt_viewer.cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

#install(TARGETS ${BINNAME1}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(TARGETS ${BINNAME2}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
