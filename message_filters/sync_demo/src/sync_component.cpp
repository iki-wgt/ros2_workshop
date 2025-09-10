#include "sync_demo/sync_component.hpp"

SyncComponent::SyncComponent(const rclcpp::NodeOptions &options)
    : rclcpp::Node("sync_component", options) {
  // parameters
  declare_parameter("topics.a", "/point_a");
  declare_parameter("topics.b", "/point_b");
  declare_parameter("sync.queue_size", 10);
  declare_parameter("sync.max_interval", 0.10); // seconds

  topic_a_ = get_parameter("topics.a").as_string();
  topic_b_ = get_parameter("topics.b").as_string();
  queue_size_ = get_parameter("sync.queue_size").as_int();
  max_interval_s_ = get_parameter("sync.max_interval").as_double();

  // subscribers
  sub_a_.subscribe(this, topic_a_, rmw_qos_profile_default);
  sub_b_.subscribe(this, topic_b_, rmw_qos_profile_default);

  sync_ = std::make_shared<ApproxSync>(ApproxPolicy(queue_size_));
  sync_->connectInput(sub_a_, sub_b_);
  sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(max_interval_s_));

  sync_->registerCallback(std::bind(&SyncComponent::syncCallback, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));

  RCLCPP_INFO(get_logger(),
              "ApproximateTime on '%s' & '%s' (queue=%d, max_interval=%.3fs)",
              topic_a_.c_str(), topic_b_.c_str(), queue_size_, max_interval_s_);
}

void SyncComponent::syncCallback(const PointMsg::ConstSharedPtr &a,
                                 const PointMsg::ConstSharedPtr &b) {
  const rclcpp::Time ta(a->header.stamp);
  const rclcpp::Time tb(b->header.stamp);
  const double dt = std::abs((ta - tb).seconds());

  // Midpoint
  const double mx = 0.5 * (a->point.x + b->point.x);
  const double my = 0.5 * (a->point.y + b->point.y);
  const double mz = 0.5 * (a->point.z + b->point.z);

  RCLCPP_INFO(
      get_logger(),
      "SYNC (|Δt|=%.6f ≤ %.6f): "
      "A(%.3f,%.3f,%.3f)@%.6f  B(%.3f,%.3f,%.3f)@%.6f  Mid(%.3f,%.3f,%.3f)",
      dt, max_interval_s_, a->point.x, a->point.y, a->point.z, ta.seconds(),
      b->point.x, b->point.y, b->point.z, tb.seconds(), mx, my, mz);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SyncComponent)
