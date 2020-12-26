cmake_minimum_required(VERSION 3.1.2)
project(mrs_mavros_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  mrs_lib
  nav_msgs
  mavros_msgs
  nodelet
  )

catkin_package(
  CATKIN_DEPENDS nav_msgs mrs_lib mavros_msgs
  LIBRARIES MavrosDiagnostics
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(MavrosDiagnostics src/mavros_diagnostics.cpp)

add_dependencies(MavrosDiagnostics
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MavrosDiagnostics
  ${catkin_LIBRARIES}
  )
