"""
Copyright 2024 Lorenzo Grandi

GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007

Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
Everyone is permitted to copy and distribute verbatim copies
of this license document, but changing it is not allowed.
"""

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/qos.hpp>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>  // For pcl::transformPointCloud

class PointCloudCombiner : public rclcpp::Node
{
public:
  PointCloudCombiner()
  : Node("point_cloud_combiner")
  {    
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Initialize the publisher
    merged_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merged_pointcloud_wrt_map", qos_profile);

    // Initialize the subscribers
    point_cloud_sub_1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_wrt_map_1", qos_profile, std::bind(&PointCloudCombiner::pointCloudCallback_1, this, std::placeholders::_1));

    point_cloud_sub_2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_wrt_map_2", qos_profile, std::bind(&PointCloudCombiner::pointCloudCallback_2, this, std::placeholders::_1));
  }

private:

  // Callback function for handling point clouds from camera 1
  void pointCloudCallback_1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    point_cloud_1_ = *msg;
    //RCLCPP_INFO(this->get_logger(), "Received point cloud from camera 1");
    point_cloud_merging();
  }

  // Callback function for handling point clouds from camera 2
  void pointCloudCallback_2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    point_cloud_2_ = *msg;
    //RCLCPP_INFO(this->get_logger(), "Received point cloud from camera 2");
    point_cloud_merging();
  }

  void point_cloud_merging ()
  {
    std::string target_frame = "map";
    
    // Check if both point clouds have been received
    if (point_cloud_1_.data.empty() || point_cloud_2_.data.empty()) {
        //RCLCPP_WARN(this->get_logger(), "One or both point clouds are empty.");
        return;
    }

    // Convert sensor_msgs::msg::PointCloud2 to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(point_cloud_1_, *pcl_cloud_1);
    pcl::fromROSMsg(point_cloud_2_, *pcl_cloud_2);
    
    // Merge the point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_merged(new pcl::PointCloud<pcl::PointXYZ>);
    *pcl_cloud_merged = *pcl_cloud_1 + *pcl_cloud_2;
    
    // Convert merged pcl::PointCloud back to sensor_msgs::PointCloud2
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*pcl_cloud_merged, output_msg);
    output_msg.header.frame_id = target_frame;

    // Publish the merged point cloud
    merged_point_cloud_pub_->publish(output_msg);
  }

  // Declare the publishers and subscribers as private member variables
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_point_cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_2_;

  // Point clouds received from each input
  sensor_msgs::msg::PointCloud2 point_cloud_1_;
  sensor_msgs::msg::PointCloud2 point_cloud_2_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudCombiner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
