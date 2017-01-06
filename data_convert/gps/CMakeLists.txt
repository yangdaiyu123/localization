cmake_minimum_required(VERSION 2.8.3)
project(gps)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include/gps
  LIBRARIES gps
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include/gps
)

add_library(gps
  src/geo_pos_conv.cpp
)

#############
## Install ##
#############

install(
  DIRECTORY include/gps
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)
