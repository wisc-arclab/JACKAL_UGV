cmake_minimum_required(VERSION 3.0.2)
project(data_for_deepvio)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  image_transport
  message_filters
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES data_for_deepvio
  CATKIN_DEPENDS roscpp message_filters sensor_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(sync_subscriber_node src/syn.cpp)

## Add cmake target dependencies of the executable
add_dependencies(sync_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(sync_subscriber_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
