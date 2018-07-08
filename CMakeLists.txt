cmake_minimum_required(VERSION 2.8.3)
project(mrs_mavros_interface)

set(CMAKE_CXX_FLAGS "-std=c++11 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  nav_msgs
  )

catkin_package(
  CATKIN_DEPENDS nav_msgs
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(mavros_interface src/mavros_interface.cpp)

add_dependencies(mavros_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(mavros_interface
  ${catkin_LIBRARIES}
  )
