cmake_minimum_required(VERSION 3.8)
project(pointcloud_combiner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

add_executable(pointcloud_combiner src/pointcloud_combiner.cpp)

# Link against the dependencies
ament_target_dependencies(pointcloud_combiner
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_eigen
)

# Install the executable
install(TARGETS
  pointcloud_combiner
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package declaration
ament_package()
