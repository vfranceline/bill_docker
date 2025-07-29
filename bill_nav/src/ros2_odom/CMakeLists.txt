cmake_minimum_required(VERSION 3.8)
project(ros2_odom)

# Encontra as dependências necessárias (pacotes ROS 2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Adiciona o executável a partir do seu arquivo .cpp
add_executable(mecanum_odometry_node src/mecanum_odometry.cpp)

# Vincula as bibliotecas ao seu executável
ament_target_dependencies(mecanum_odometry_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

# Instala o executável para que você possa rodá-lo com "ros2 run"
install(TARGETS
  mecanum_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# Registra o pacote no ament
ament_package()
