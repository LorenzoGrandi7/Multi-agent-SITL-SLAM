cmake_minimum_required(VERSION 3.5)
project(pointcloud_transformer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(pointcloud_transformer src/pointcloud_transformer.cpp)

ament_target_dependencies(pointcloud_transformer
  rclcpp
  sensor_msgs
  pcl_ros
  tf2_ros
  geometry_msgs
  px4_msgs
)

install(TARGETS
  pointcloud_transformer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

