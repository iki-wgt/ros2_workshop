#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

// -------------------- Talker Component --------------------
class TalkerComponent : public rclcpp::Node {
public:
  explicit TalkerComponent(const rclcpp::NodeOptions &options)
      : rclcpp::Node("zero_copy_talker", options) {
    // Params
    this->declare_parameter<std::string>("topic", "chatter");
    this->declare_parameter<double>("publish_rate_hz", 2.0);

    topic_ = this->get_parameter("topic").as_string();
    rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    pub_ = this->create_publisher<std_msgs::msg::String>(
        topic_, rclcpp::QoS(10).best_effort());

    auto period = std::chrono::duration<double>(1.0 / std::max(0.1, rate_hz_));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&TalkerComponent::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Talker publishing on '%s' at %.2f Hz",
                topic_.c_str(), rate_hz_);
  }

private:
  void onTimer() {
    // Allocate message with unique_ptr for zero-copy transport (when
    // intra-process is enabled).
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "hello " + std::to_string(count_++);

    // Log the message content and its memory address before publishing.
    RCLCPP_INFO(this->get_logger(),
                "Sending message with data: %s, at address: 0x%" PRIxPTR,
                msg->data.c_str(), reinterpret_cast<std::uintptr_t>(msg.get()));

    // Publish by moving the unique_ptr (zero-copy intra-process).
    pub_->publish(std::move(msg));
  }

  std::string topic_;
  double rate_hz_{2.0};
  uint64_t count_{0};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(TalkerComponent)

// -------------------- Listener Component --------------------
class ListenerComponent : public rclcpp::Node {
public:
  explicit ListenerComponent(const rclcpp::NodeOptions &options)
      : rclcpp::Node("zero_copy_listener", options) {
    // Params
    this->declare_parameter<std::string>("topic", "chatter");
    topic_ = this->get_parameter("topic").as_string();

    // Subscription uses unique_ptr to enable zero-copy intra-process delivery.
    sub_ = this->create_subscription<std_msgs::msg::String>(
        topic_, rclcpp::QoS(10).best_effort(),
        std::bind(&ListenerComponent::onMsg, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Listener subscribed to '%s'",
                topic_.c_str());
  }

private:
  // Note the signature: std::unique_ptr<T>
  void onMsg(std::unique_ptr<std_msgs::msg::String> msg) {
    // Log the received message data and its memory address.
    RCLCPP_INFO(this->get_logger(),
                "Received message with data: %s, at address: 0x%" PRIxPTR,
                msg->data.c_str(), reinterpret_cast<std::uintptr_t>(msg.get()));
  }

  std::string topic_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ListenerComponent)
