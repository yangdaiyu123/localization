cmake_minimum_required(VERSION 2.8.3)
project(motion_correct)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	sensor_msgs

  	pcl_conversions
  	pcl_ros
	curb_detector
)

find_package(PCL 1.7 REQUIRED common io visualization)

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
  INCLUDE_DIRS include/motion_correct
  LIBRARIES motion_correct
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
  sensor_msgs
  curb_detector
)

include_directories(
	include ${catkin_INCLUDE_DIRS}
	include ${roscpp_INCLUDE_DIRS}
	include/${PROJECT_NAME}/
)

# Set the name to use for the executable.

## Set the source files to use with the executable.

# Build the executable that will be used to run this node.

add_library(motion_correct src/motion_correct.cpp)
#target_link_libraries(curb_detector ransac_line)

add_executable(motion_correct_node src/motion_correct_node.cpp)
target_link_libraries(motion_correct_node ${catkin_LIBRARIES})

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
