"""
Copyright 2024 Lorenzo Grandi

GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007

Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
Everyone is permitted to copy and distribute verbatim copies
of this license document, but changing it is not allowed.
"""

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/qos.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <pcl/filters/passthrough.h>

class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer()
  : Node("point_cloud_transformer")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        
    rclcpp::QoS qos_profile_2 = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    point_cloud_pub_1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_wrt_map_1", qos_profile);
    point_cloud_pub_2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_wrt_map_2", qos_profile);

    point_cloud_sub_1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera_1/points", qos_profile, std::bind(&PointCloudTransformer::pointCloudCallback_1, this, std::placeholders::_1));

    point_cloud_sub_2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera_2/points", qos_profile, std::bind(&PointCloudTransformer::pointCloudCallback_2, this, std::placeholders::_1));
      
    vehicle_attitude_sub_1 = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/px4_1/fmu/out/vehicle_attitude", qos_profile_2, std::bind(&PointCloudTransformer::vehicleAttitudeCallback_1, this, std::placeholders::_1));
      
    vehicle_attitude_sub_2 = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/px4_2/fmu/out/vehicle_attitude", qos_profile_2, std::bind(&PointCloudTransformer::vehicleAttitudeCallback_2, this, std::placeholders::_1));
  }

private:

  std::mutex callback_mutex_;
  tf2::Quaternion last_vehicle_rotation_1;
  tf2::Quaternion last_vehicle_rotation_2;
  double vehicle_rotation_angle_diff_1 = 0.0;
  double vehicle_rotation_angle_diff_2 = 0.0;
  bool drone_rotating_1 = false;
  bool drone_rotating_2 = false;
  double rotation_threshold = 0.00174532925;         // 0.1° rotation
  
  // Callback function for handling point clouds from camera 1
  void pointCloudCallback_1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(callback_mutex_); // Ensure thread-safety
    std::string target_frame = "map";

    try
    {
      // Transform the point cloud to the 'map' frame
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
        target_frame, msg->header.frame_id, tf2::TimePointZero);
        
      RCLCPP_INFO(this->get_logger(), "Drone 1 angle: %f",  vehicle_rotation_angle_diff_1);
        
      if (drone_rotating_1) {
        // Publish an empty point cloud if the drone is moving
        sensor_msgs::msg::PointCloud2 empty_msg;
        empty_msg.header.frame_id = target_frame;
        empty_msg.header.stamp = msg->header.stamp;
        point_cloud_pub_1_->publish(empty_msg);
        return;
      }

      sensor_msgs::msg::PointCloud2 transformed_msg;
      pcl_ros::transformPointCloud(target_frame, transform_stamped, *msg, transformed_msg);

      // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(transformed_msg, *pcl_cloud);
      
      // Apply PassThrough filter to remove ground points
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(pcl_cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.2, std::numeric_limits<float>::max()); // Keep points with z >= 0.2
      pass.filter(*filtered_cloud);

      // Apply VoxelGrid filter for downsampling
      pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud(filtered_cloud);
      voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);  // Adjust the leaf size as needed
      voxel_filter.filter(*downsampled_cloud);

      // Convert filtered pcl::PointCloud back to sensor_msgs::PointCloud2
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*downsampled_cloud, output_msg);
      output_msg.header.frame_id = target_frame;
      output_msg.header.stamp = msg->header.stamp;

      // Publish the filtered point cloud
      point_cloud_pub_1_->publish(output_msg);
    }
    catch (tf2::TransformException & ex)
    {
      //RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    }
  }

  // Callback function for handling point clouds from camera 2
  void pointCloudCallback_2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  { 
    std::lock_guard<std::mutex> lock(callback_mutex_); // Ensure thread-safety
    std::string target_frame = "map";

    try
    {
      // Transform the point cloud to the 'map' frame
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
        target_frame, msg->header.frame_id, tf2::TimePointZero);
        
      RCLCPP_INFO(this->get_logger(), "Drone 2 angle: %f",  vehicle_rotation_angle_diff_2);
        
      if (drone_rotating_2) {
        // Publish an empty point cloud if the drone is moving
        sensor_msgs::msg::PointCloud2 empty_msg;
        empty_msg.header.frame_id = target_frame;
        empty_msg.header.stamp = msg->header.stamp;
        point_cloud_pub_2_->publish(empty_msg);
        return;
      }

      sensor_msgs::msg::PointCloud2 transformed_msg;
      pcl_ros::transformPointCloud(target_frame, transform_stamped, *msg, transformed_msg);

      // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(transformed_msg, *pcl_cloud);
      
      // Apply PassThrough filter to remove ground points
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(pcl_cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.2, std::numeric_limits<float>::max()); // Keep points with z >= 0.2
      pass.filter(*filtered_cloud);

      // Apply VoxelGrid filter for downsampling
      pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud(filtered_cloud);
      voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);  // Adjust the leaf size as needed
      voxel_filter.filter(*downsampled_cloud);

      // Convert filtered pcl::PointCloud back to sensor_msgs::PointCloud2
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*downsampled_cloud, output_msg);
      output_msg.header.frame_id = target_frame;
      output_msg.header.stamp = msg->header.stamp;

      // Publish the filtered point cloud
      point_cloud_pub_2_->publish(output_msg);
    }
    catch (tf2::TransformException & ex)
    {
    //  RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    }
  }
  
  void vehicleAttitudeCallback_1(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    // Extract quaternion orientation
    tf2::Quaternion current_vehicle_rotation(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    
    // Normalize the quaternion to ensure validity
    current_vehicle_rotation.normalize();
    
    // Initialize `last_vehicle_rotation_1` if it's uninitialized
    if (last_vehicle_rotation_1.length2() == 0) {
        last_vehicle_rotation_1 = current_vehicle_rotation;
        return; // Skip angle calculation for the first callback
    }

    // Calculate the angular difference
    double angle_diff = last_vehicle_rotation_1.angleShortestPath(current_vehicle_rotation);

    // Update the last rotation
    last_vehicle_rotation_1 = current_vehicle_rotation;

    // Store the angular difference for use in point cloud callbacks
    vehicle_rotation_angle_diff_1 = angle_diff; // A member variable to store the difference
    
    // Check if the angle is within a settled threshold
    if (std::abs(angle_diff) > rotation_threshold) {
        drone_rotating_1 = true;
    } else {
        drone_rotating_1 = false;
    }
  }
  
  void vehicleAttitudeCallback_2(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    // Extract quaternion orientation
    tf2::Quaternion current_vehicle_rotation(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    
    // Normalize the quaternion to ensure validity
    current_vehicle_rotation.normalize();
    
    // Initialize `last_vehicle_rotation_1` if it's uninitialized
    if (last_vehicle_rotation_2.length2() == 0) {
        last_vehicle_rotation_2 = current_vehicle_rotation;
        return; // Skip angle calculation for the first callback
    }

    // Calculate the angular difference
    double angle_diff = last_vehicle_rotation_2.angleShortestPath(current_vehicle_rotation);

    // Update the last rotation
    last_vehicle_rotation_2 = current_vehicle_rotation;

    // Store the angular difference for use in point cloud callbacks
    vehicle_rotation_angle_diff_2 = angle_diff; // A member variable to store the difference
    
    // Check if the angle is within a settled threshold
    if (std::abs(angle_diff) > rotation_threshold) {
        drone_rotating_2 = true;
    } else {
        drone_rotating_2 = false;
    }
  }

  // Declare the publishers and subscribers as private member variables
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_1_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_2_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_2_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_1;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_2;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudTransformer>();

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
