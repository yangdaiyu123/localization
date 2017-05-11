cmake_minimum_required(VERSION 2.8.3)
project(point_transform)

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
	cfg/top_transform.cfg
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
)

# Set the name to use for the executable.
set (BINNAME1 front_transform)
set (BINNAME2 rear_transform)
set (BINNAME3 point_to_base_link)
set (BINNAME4 points_to_stixel)

## Set the source files to use with the executable.
set (SRCS1 ${SRCS1} src/front_transform.cpp)
set (SRCS2 ${SRCS2} src/rear_transform.cpp)
set (SRCS3 ${SRCS3} src/point_to_base_link.cpp)
set (SRCS4 ${SRCS4} src/points_to_stixel.cpp)

# Build the executable that will be used to run this node.
add_executable (${BINNAME1} ${SRCS1})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME1} ${PCL_LIBRARIES})

add_executable (${BINNAME2} ${SRCS2})
target_link_libraries(${BINNAME2} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME2} ${PCL_LIBRARIES})

add_executable (${BINNAME4} ${SRCS4})
target_link_libraries(${BINNAME4} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME4} ${PCL_LIBRARIES})

#add_executable (${BINNAME3} ${SRCS3})
#target_link_libraries(${BINNAME3} ${catkin_LIBRARIES})
#target_link_libraries(${BINNAME3} ${PCL_LIBRARIES})
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
