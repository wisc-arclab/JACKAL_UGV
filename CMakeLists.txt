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
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  StampedModelStates.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES data_for_deepvio
  CATKIN_DEPENDS roscpp message_filters sensor_msgs nav_msgs message_runtime geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(sync_node src/syn.cpp)
add_dependencies(sync_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sync_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(model_state_stamper src/model_state_stamper.cpp)
target_link_libraries(model_state_stamper ${catkin_LIBRARIES})
add_dependencies(model_state_stamper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
