cmake_minimum_required(VERSION 3.0.2)
project(keyboard_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES keyboard_control
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_control src/keyboard_control.cpp)

target_link_libraries(keyboard_control
  ${catkin_LIBRARIES}
)

add_executable(drive_robot src/drive_robot.cpp)
target_link_libraries(drive_robot ${catkin_LIBRARIES})
