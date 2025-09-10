#pragma once

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class DemoComponent : public rclcpp::Node {
public:
  explicit DemoComponent(const rclcpp::NodeOptions &options);

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  double leaf_size_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};
