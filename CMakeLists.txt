cmake_minimum_required(VERSION 3.0.2)
project(acados_mpc)

add_compile_options(-std=c++11 -O3)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  cv_bridge
  tf
  tf2
  
  std_msgs
  visualization_msgs
  message_generation
  sensor_msgs
)

set(acados_include "/home/pc2004/acados/include")
set(acados_lib "/home/pc2004/acados/lib")

include_directories(
include
/usr/local/include
src/Eigen-3.3
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
scripts/SingleIntegrator/c_generated_code
${PROJECT_SOURCE_DIR}/scripts/SingleIntegrator/c_generated_code/bicycle_model_model
${PROJECT_SOURCE_DIR}/scripts/SingleIntegrator/c_generated_code/bicycle_model_cost
${PROJECT_SOURCE_DIR}/scripts/SingleIntegrator/c_generated_code
${acados_include}
${acados_include}/blasfeo/include
${acados_include}/hpipm/include
${acados_include}/acados/
${acados_include}/qpOASES_e
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES acados_mpc
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf2 message_runtime
#  DEPENDS system_lib
)

link_directories(${PROJECT_SOURCE_DIR}/scripts/c_generated_code)
link_directories(/usr/local/lib)

add_executable(mpc_node src/mpc_node.cpp)
target_link_libraries(mpc_node
	${PROJECT_SOURCE_DIR}/scripts/SingleIntegrator/c_generated_code/libacados_ocp_solver_SingleIntegrator.so
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

add_executable(trajectory_publisher src/trajectory_publisher.cpp)
target_link_libraries(trajectory_publisher ${catkin_LIBRARIES})
add_dependencies(trajectory_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})
add_dependencies(odom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
