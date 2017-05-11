cmake_minimum_required(VERSION 2.8.3)
project(pcl_demo)

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
  cfg/section.cfg
  cfg/grid.cfg
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
set (BINNAME1 write_pcd)
set (BINNAME2 load_pcd)
set (BINNAME3 ndt_demo)
set (BINNAME4 load_pcd_section)
set (BINNAME5 write_pcd_by_odom)
set (BINNAME6 write_pcd_by_grid)
set (BINNAME7 load_pcd_grid)
set (BINNAME8 icp_demo)
set (BINNAME9 show_pcd)
set (BINNAME10 show_pcd_transform)
set (BINNAME11 icp_demo2)
set (BINNAME12 ndt_demo2)
set (BINNAME13 point_xyzo)
set (BINNAME14 icp_demo3)
set (BINNAME15 write_oriented_pcd)
set (BINNAME16 write_curb_map)
set (BINNAME17 load_pcd_odom)
set (BINNAME18 write_pcd_odom)
set (BINNAME19 point_cloud_sync)

## Set the source files to use with the executable.
set (SRCS1 ${SRCS1} src/write_pcd.cpp)
set (SRCS2 ${SRCS2} src/load_pcd.cpp)
set (SRCS3 ${SRCS3} src/ndt_demo.cpp)
set (SRCS4 ${SRCS4} src/load_pcd_section.cpp)
set (SRCS5 ${SRCS5} src/write_pcd_by_odom.cpp)
set (SRCS6 ${SRCS6} src/write_pcd_by_grid.cpp)
set (SRCS7 ${SRCS7} src/load_pcd_grid.cpp)
set (SRCS8 ${SRCS8} src/icp_demo.cpp)
set (SRCS9 ${SRCS9} src/show_pcd.cpp)
set (SRCS10 ${SRCS10} src/show_pcd_transform.cpp)
set (SRCS11 ${SRCS11} src/icp_demo2.cpp)
set (SRCS12 ${SRCS12} src/ndt_demo2.cpp)
set (SRCS13 ${SRCS13} src/point_xyzo.cpp)
set (SRCS14 ${SRCS14} src/icp_demo3.cpp)
set (SRCS15 ${SRCS15} src/write_oriented_pcd.cpp)
set (SRCS16 ${SRCS16} src/write_curb_map.cpp)
set (SRCS17 ${SRCS17} src/load_pcd_odom.cpp)
set (SRCS18 ${SRCS18} src/write_pcd_odom.cpp)
set (SRCS19 ${SRCS19} src/point_cloud_sync.cpp)

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

add_executable (${BINNAME4} ${SRCS4})
target_link_libraries(${BINNAME4} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME4} ${PCL_LIBRARIES})

add_executable (${BINNAME5} ${SRCS5})
target_link_libraries(${BINNAME5} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME5} ${PCL_LIBRARIES})

add_executable (${BINNAME6} ${SRCS6})
target_link_libraries(${BINNAME6} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME6} ${PCL_LIBRARIES})

add_executable (${BINNAME7} ${SRCS7})
target_link_libraries(${BINNAME7} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME7} ${PCL_LIBRARIES})

add_executable (${BINNAME8} ${SRCS8})
target_link_libraries(${BINNAME8} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME8} ${PCL_LIBRARIES})

add_executable (${BINNAME9} ${SRCS9})
target_link_libraries(${BINNAME9} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME9} ${PCL_LIBRARIES})

add_executable (${BINNAME10} ${SRCS10})
target_link_libraries(${BINNAME10} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME10} ${PCL_LIBRARIES})

add_executable (${BINNAME11} ${SRCS11})
target_link_libraries(${BINNAME11} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME11} ${PCL_LIBRARIES})

add_executable (${BINNAME12} ${SRCS12})
target_link_libraries(${BINNAME12} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME12} ${PCL_LIBRARIES})

add_executable (${BINNAME13} ${SRCS13})
target_link_libraries(${BINNAME13} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME13} ${PCL_LIBRARIES})

add_executable (${BINNAME14} ${SRCS14})
target_link_libraries(${BINNAME14} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME14} ${PCL_LIBRARIES})

add_executable (${BINNAME15} ${SRCS15})
target_link_libraries(${BINNAME15} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME15} ${PCL_LIBRARIES})

add_executable (${BINNAME16} ${SRCS16})
target_link_libraries(${BINNAME16} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME16} ${PCL_LIBRARIES})

add_executable (${BINNAME17} ${SRCS17})
target_link_libraries(${BINNAME17} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME17} ${PCL_LIBRARIES})

add_executable (${BINNAME18} ${SRCS18})
target_link_libraries(${BINNAME18} ${catkin_LIBRARIES})
target_link_libraries(${BINNAME18} ${PCL_LIBRARIES})

#add_executable (${BINNAME19} ${SRCS19})
#target_link_libraries(${BINNAME19} ${catkin_LIBRARIES})
#target_link_libraries(${BINNAME19} ${PCL_LIBRARIES})

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
