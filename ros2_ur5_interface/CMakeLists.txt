cmake_minimum_required(VERSION 3.8)
project(ros2_ur5_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)  # Aggiungi questa linea per il messaggio geometry_msgs
# Trova la libreria Eigen3
find_package(Eigen3 REQUIRED)

# Aggiungi le dipendenze alla compilazione
include_directories(${EIGEN3_INCLUDE_DIR})


# Aggiungi l'eseguibile per il nodo inverse_kin_exec
add_executable(inverse_kin_exec src/inverse_kin.cpp)
add_executable(publish_trajectory_node src/publish_trajectory_node.cpp)
add_executable(gripper_service src/gripper_service.cpp)

ament_target_dependencies(publish_trajectory_node
  rclcpp
  trajectory_msgs
  sensor_msgs
  control_msgs
  rclcpp_action
  std_srvs
)

ament_target_dependencies(gripper_service
  rclcpp
  ament_index_cpp
  trajectory_msgs
  sensor_msgs
  control_msgs
  rclcpp_action
  std_srvs
)

ament_target_dependencies(inverse_kin_exec  # Aggiungi le dipendenze per inverse_kin_exec
  rclcpp
  geometry_msgs  # Aggiungi le dipendenze che hai utilizzato nel file inverse_kin.cpp
)

install(TARGETS
  publish_trajectory_node
  gripper_service
  inverse_kin_exec  # Aggiungi inverse_kin_exec per l'installazione
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(
  DIRECTORY 
  launch
  config
  params
  gripper
  worlds
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
