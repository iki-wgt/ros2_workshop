#include "sensor_calibration/interactive_tf_calib.hpp"

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::InteractiveMarkerFeedback;
using visualization_msgs::msg::Marker;

InteractiveTfCalib::InteractiveTfCalib(const rclcpp::NodeOptions &options)
    : rclcpp::Node("interactive_tf_calib", options) {
  // Parameters
  declare_parameter<std::string>("frames.map", "map");
  declare_parameter<std::string>("frames.child_a", "sensor_a");
  declare_parameter<std::string>("frames.child_b", "sensor_b");

  declare_parameter<double>("publish_rate_hz", 30.0);

  // Initial poses (position + RPY) for both children
  declare_parameter<double>("poses.a.x", 0.0);
  declare_parameter<double>("poses.a.y", 0.0);
  declare_parameter<double>("poses.a.z", 0.0);
  declare_parameter<double>("poses.a.roll", 0.0);
  declare_parameter<double>("poses.a.pitch", 0.0);
  declare_parameter<double>("poses.a.yaw", 0.0);

  declare_parameter<double>("poses.b.x", 1.0);
  declare_parameter<double>("poses.b.y", 0.0);
  declare_parameter<double>("poses.b.z", 0.0);
  declare_parameter<double>("poses.b.roll", 0.0);
  declare_parameter<double>("poses.b.pitch", 0.0);
  declare_parameter<double>("poses.b.yaw", 0.0);

  map_frame_ = get_parameter("frames.map").as_string();
  child_a_ = get_parameter("frames.child_a").as_string();
  child_b_ = get_parameter("frames.child_b").as_string();
  pub_rate_hz_ = get_parameter("publish_rate_hz").as_double();

  // Build initial poses from params
  auto pose_a = rpyToPose(get_parameter("poses.a.x").as_double(),
                          get_parameter("poses.a.y").as_double(),
                          get_parameter("poses.a.z").as_double(),
                          get_parameter("poses.a.roll").as_double(),
                          get_parameter("poses.a.pitch").as_double(),
                          get_parameter("poses.a.yaw").as_double());

  auto pose_b = rpyToPose(get_parameter("poses.b.x").as_double(),
                          get_parameter("poses.b.y").as_double(),
                          get_parameter("poses.b.z").as_double(),
                          get_parameter("poses.b.roll").as_double(),
                          get_parameter("poses.b.pitch").as_double(),
                          get_parameter("poses.b.yaw").as_double());

  // Prepare TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Initialize transforms (map -> child)
  t_map_a_.header.frame_id = map_frame_;
  t_map_a_.child_frame_id = child_a_;
  t_map_a_.transform.translation.x = pose_a.position.x;
  t_map_a_.transform.translation.y = pose_a.position.y;
  t_map_a_.transform.translation.z = pose_a.position.z;
  t_map_a_.transform.rotation = pose_a.orientation;

  t_map_b_.header.frame_id = map_frame_;
  t_map_b_.child_frame_id = child_b_;
  t_map_b_.transform.translation.x = pose_b.position.x;
  t_map_b_.transform.translation.y = pose_b.position.y;
  t_map_b_.transform.translation.z = pose_b.position.z;
  t_map_b_.transform.rotation = pose_b.orientation;

  // Interactive Marker server
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "tf_calibration_server", get_node_base_interface(),
      get_node_clock_interface(), get_node_logging_interface(),
      get_node_topics_interface(), get_node_services_interface());

  // Create 6-DOF markers for both frames (in map frame)
  auto im_a = make6DofMarker(child_a_, pose_a);
  im_a.header.frame_id = map_frame_;
  server_->insert(im_a, std::bind(&InteractiveTfCalib::onFeedback, this,
                                  std::placeholders::_1));

  auto im_b = make6DofMarker(child_b_, pose_b);
  im_b.header.frame_id = map_frame_;
  server_->insert(im_b, std::bind(&InteractiveTfCalib::onFeedback, this,
                                  std::placeholders::_1));

  server_->applyChanges();

  // TF publish timer
  auto period =
      std::chrono::duration<double>(1.0 / std::max(1.0, pub_rate_hz_));
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&InteractiveTfCalib::onTimer, this));

  RCLCPP_INFO(
      get_logger(),
      "Interactive TF Calib started. map='%s', A='%s', B='%s' (rate=%.1f Hz)",
      map_frame_.c_str(), child_a_.c_str(), child_b_.c_str(), pub_rate_hz_);
}

InteractiveMarker
InteractiveTfCalib::make6DofMarker(const std::string &name,
                                   const geometry_msgs::msg::Pose &pose) {
  InteractiveMarker im;
  im.name = name;
  im.description = name + " (drag to move/rotate)";
  im.scale = 0.4;
  im.pose = pose;

  // A visible box
  Marker box;
  box.type = Marker::CUBE;
  box.scale.x = 0.15;
  box.scale.y = 0.15;
  box.scale.z = 0.15;
  box.color.r = 0.2f;
  box.color.g = 0.6f;
  box.color.b = 1.0f;
  box.color.a = 0.8f;

  InteractiveMarkerControl box_ctrl;
  box_ctrl.always_visible = true;
  box_ctrl.markers.push_back(box);
  im.controls.push_back(box_ctrl);

  // Helper to add one axis of rotation+move
  auto add_axis = [&](double x, double y, double z, const std::string &suffix) {
    InteractiveMarkerControl rot;
    rot.orientation.w = 1.0;
    rot.orientation.x = x;
    rot.orientation.y = y;
    rot.orientation.z = z;
    rot.name = "rotate_" + suffix;
    rot.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    im.controls.push_back(rot);

    InteractiveMarkerControl move;
    move.orientation = rot.orientation;
    move.name = "move_" + suffix;
    move.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    im.controls.push_back(move);
  };

  add_axis(1, 0, 0, "x");
  add_axis(0, 1, 0, "y");
  add_axis(0, 0, 1, "z");

  // Free 3D move/rotate control (nice for coarse alignment)
  InteractiveMarkerControl move3d;
  move3d.name = "move_rotate_3d";
  move3d.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D;
  im.controls.push_back(move3d);

  return im;
}

geometry_msgs::msg::Pose InteractiveTfCalib::rpyToPose(double x, double y,
                                                       double z, double roll,
                                                       double pitch,
                                                       double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

void InteractiveTfCalib::onFeedback(
    const InteractiveMarkerFeedback::ConstSharedPtr &fb) {
  // Update pose in corresponding transform
  if (fb->marker_name == child_a_) {
    t_map_a_.transform.translation.x = fb->pose.position.x;
    t_map_a_.transform.translation.y = fb->pose.position.y;
    t_map_a_.transform.translation.z = fb->pose.position.z;
    t_map_a_.transform.rotation = fb->pose.orientation;
  } else if (fb->marker_name == child_b_) {
    t_map_b_.transform.translation.x = fb->pose.position.x;
    t_map_b_.transform.translation.y = fb->pose.position.y;
    t_map_b_.transform.translation.z = fb->pose.position.z;
    t_map_b_.transform.rotation = fb->pose.orientation;
  }
  // Reflect pose in the server (keeps RViz display consistent)
  server_->setPose(fb->marker_name, fb->pose);
  server_->applyChanges();
}

void InteractiveTfCalib::onTimer() {
  const auto now = get_clock()->now();
  t_map_a_.header.stamp = now;
  t_map_b_.header.stamp = now;
  tf_broadcaster_->sendTransform(t_map_a_);
  tf_broadcaster_->sendTransform(t_map_b_);
}

RCLCPP_COMPONENTS_REGISTER_NODE(InteractiveTfCalib)
