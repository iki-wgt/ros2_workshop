#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class SyncComponent : public rclcpp::Node {
public:
  explicit SyncComponent(const rclcpp::NodeOptions &options);

private:
  using PointMsg = geometry_msgs::msg::PointStamped;
  using ApproxPolicy =
      message_filters::sync_policies::ApproximateTime<PointMsg, PointMsg>;
  using ApproxSync = message_filters::Synchronizer<ApproxPolicy>;

  void syncCallback(const PointMsg::ConstSharedPtr &a,
                    const PointMsg::ConstSharedPtr &b);

  // params
  std::string topic_a_;
  std::string topic_b_;
  int queue_size_{10};
  double max_interval_s_{0.10};

  // subs + sync
  message_filters::Subscriber<PointMsg> sub_a_;
  message_filters::Subscriber<PointMsg> sub_b_;
  std::shared_ptr<ApproxSync> sync_;
};
