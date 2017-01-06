cmake_minimum_required(VERSION 2.8.3)
project(octomap_viewer)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	sensor_msgs

	pcl_conversions
  	pcl_ros
)


find_package(PCL 1.7 REQUIRED common io visualization)
find_package(octomap REQUIRED)


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
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
  sensor_msgs
)

include_directories(
	include ${catkin_INCLUDE_DIRS}
	include ${roscpp_INCLUDE_DIRS}
	${mrpt_INCLUDE_DIRS}
	include/${PROJECT_NAME}/
)
include_directories(${OCTOMAP_INCLUDE_DIRS})

# Set the name to use for the executable.

## Set the source files to use with the executable.

# Build the executable that will be used to run this node.

add_executable(octomap_viewer src/octomap_viewer.cpp)
target_link_libraries(octomap_viewer ${catkin_LIBRARIES})
target_link_libraries(octomap_viewer ${PCL_LIBRARIES})
target_link_libraries(octomap_viewer ${OCTOMAP_LIBRARIES})


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
