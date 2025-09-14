#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloud_segmentation {

class SegmentationComponent : public rclcpp::Node {
public:
  explicit SegmentationComponent(const rclcpp::NodeOptions &options);

private:
  using Cloud = sensor_msgs::msg::PointCloud2;
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<Cloud, Cloud>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  struct Params {
    bool use_sim_time{false};

    struct Topics {
      std::string cloud_a{"/cloud_a"};
      std::string cloud_b{"/cloud_b"};
      std::string output{"/segmented_cloud"};
    } topics;

    struct Frames {
      std::string map{"map"};
    } frames;

    struct Filters {
      double voxel_leaf{0.1};
      struct Z {
        double min{-1.0};
        double max{std::numeric_limits<double>::infinity()};
      } z;
    } filters;

    struct Clustering {
      double tolerance{0.4};
      int min_size{30};
      int max_size{250000};
    } clustering;

    struct SyncCfg {
      int queue_size{10};
      double slop{0.05};
    } sync;

  } params_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // IO
  std::shared_ptr<message_filters::Subscriber<Cloud>> sub_a_;
  std::shared_ptr<message_filters::Subscriber<Cloud>> sub_b_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Publisher<Cloud>::SharedPtr pub_;

  // Callback
  void syncedCallback(const Cloud::ConstSharedPtr &a,
                      const Cloud::ConstSharedPtr &b);

  // Helpers
  bool lookupAndTransform(const Cloud::ConstSharedPtr &in, Cloud &out) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr toPclXYZ(const Cloud &msg) const;
  Cloud::UniquePtr toRosMsg(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                            const std_msgs::msg::Header &header) const;

  // Param handling
  void declareParameters();
  void readParameters();
};

} // namespace pointcloud_segmentation

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_segmentation::SegmentationComponent)
