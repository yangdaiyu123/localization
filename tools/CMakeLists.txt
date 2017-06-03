cmake_minimum_required(VERSION 2.8.3)
project(tools)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	sensor_msgs

  	pcl_conversions
  	pcl_ros
)

find_package(PCL 1.7 REQUIRED common io visualization)

#add_message_files(
#  	FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/lidar_calibration.cfg
)

catkin_package(
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
)

# Set the name to use for the executable.
set (BINNAME1 lidar_calibration)
set (BINNAME2 sync_calibration)

## Set the source files to use with the executable.
set (SRCS1 ${SRCS1} src/lidar_calibration.cpp)
set (SRCS2 ${SRCS2} src/sync_calibration.cpp)

# Build the executable that will be used to run this node.
add_executable (${BINNAME1} ${SRCS1})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME1} ${PCL_LIBRARIES})

add_executable (sync_lidar_calibration src/sync_calibration.cpp)
target_link_libraries(sync_lidar_calibration ${catkin_LIBRARIES})
target_link_libraries(sync_lidar_calibration ${PCL_LIBRARIES})

add_executable (talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable (listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

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
