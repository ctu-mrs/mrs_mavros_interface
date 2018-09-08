cmake_minimum_required(VERSION 2.8.3)
project(mrs_mavros_interface)

set(CMAKE_CXX_FLAGS "-std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  mrs_lib
  nav_msgs
  nodelet
  )

catkin_package(
  CATKIN_DEPENDS nav_msgs mrs_lib
  LIBRARIES MavrosInterface MavrosDiagnostics
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Declare a C++ executable
add_library(MavrosInterface src/mavros_interface.cpp)

add_dependencies(MavrosInterface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MavrosInterface
  ${catkin_LIBRARIES}
  )

add_library(MavrosDiagnostics src/mavros_diagnostics.cpp)

add_dependencies(MavrosDiagnostics
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MavrosDiagnostics
  ${catkin_LIBRARIES}
  )
