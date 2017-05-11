cmake_minimum_required(VERSION 2.8.3)
project(display_features)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	sensor_msgs

  	pcl_conversions
  	pcl_ros
  	
	cv_bridge
	image_transport

)

find_package(PCL 1.7 REQUIRED common io visualization)

#add_message_files(
#  	FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#generate_dynamic_reconfigure_options(

#)

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
set (BINNAME1 cloud_sum_in_base_link)
set (BINNAME2 display_fitting_curb)
set (BINNAME3 road_marking_process)
set (BINNAME4 display_sign)
set (BINNAME5 road_marking_pointcloud)
set (BINNAME6 display_pcd)



# Set the source files to use with the executable.
set (SRCS1 ${SRCS1} src/cloud_sum_in_base_link.cpp)
set (SRCS2 ${SRCS2} src/display_fitting_curb.cpp)
set (SRCS3 ${SRCS3} src/road_marking_process.cpp)
set (SRCS4 ${SRCS4} src/display_sign.cpp)
set (SRCS5 ${SRCS5} src/road_marking_pointcloud.cpp)
set (SRCS6 ${SRCS6} src/display_pcd.cpp)


# Build the executable that will be used to run this node.
add_executable (${BINNAME1} ${SRCS1})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME1} ${PCL_LIBRARIES})

add_executable (${BINNAME2} ${SRCS2})
target_link_libraries(${BINNAME2} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME2} ${PCL_LIBRARIES})

add_executable (${BINNAME3} ${SRCS3})
target_link_libraries(${BINNAME3} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME3} ${PCL_LIBRARIES})
target_link_libraries(${BINNAME3} ${OpenCV_LIBRARIES})

add_executable (${BINNAME4} ${SRCS4})
target_link_libraries(${BINNAME4} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME4} ${PCL_LIBRARIES})

add_executable (${BINNAME5} ${SRCS5})
target_link_libraries(${BINNAME5} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME5} ${PCL_LIBRARIES})

add_executable (${BINNAME6} ${SRCS6})
target_link_libraries(${BINNAME6} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME6} ${PCL_LIBRARIES})
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
