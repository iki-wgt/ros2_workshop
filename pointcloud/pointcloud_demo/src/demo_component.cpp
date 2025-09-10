#include "pointcloud_demo/demo_component.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

DemoComponent::DemoComponent(const rclcpp::NodeOptions &options)
    : Node("demo_component", options) {
  declare_parameter("leaf_size", 0.1);

  leaf_size_ = get_parameter("leaf_size").as_double();

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", rclcpp::SensorDataQoS(),
      std::bind(&DemoComponent::cloudCallback, this, std::placeholders::_1));

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out",
                                                         rclcpp::QoS(10));
  RCLCPP_INFO(get_logger(), "Starting with leaf_size = %.3f", leaf_size_);
}

void DemoComponent::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>());
  voxel.filter(*cloud_filtered);

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud_filtered, output);
  output.header = msg->header;
  pub_->publish(output);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DemoComponent)
