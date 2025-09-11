#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>

class InteractiveTfCalib : public rclcpp::Node {
public:
  explicit InteractiveTfCalib(const rclcpp::NodeOptions &options);

private:
  // Helpers
  visualization_msgs::msg::InteractiveMarker
  make6DofMarker(const std::string &name, const geometry_msgs::msg::Pose &pose);

  static geometry_msgs::msg::Pose rpyToPose(double x, double y, double z,
                                            double roll, double pitch,
                                            double yaw);

  // Feedback from RViz: update stored transforms
  void onFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &fb);

  // Broadcast TFs @ fixed rate
  void onTimer();

  // Params
  std::string map_frame_;
  std::string child_a_;
  std::string child_b_;
  double pub_rate_hz_{30.0};

  // State (current transforms from map -> child)
  geometry_msgs::msg::TransformStamped t_map_a_;
  geometry_msgs::msg::TransformStamped t_map_b_;

  // IM server + TF broadcaster
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};
