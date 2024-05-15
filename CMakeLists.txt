cmake_minimum_required(VERSION 2.8.3)
project(mavros_diagnostic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_msgs
  mavros_msgs
  )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(mavros_diagnostic src/mavros_diagnostic.cpp)

target_link_libraries(mavros_diagnostic
  ${catkin_LIBRARIES}
  )

