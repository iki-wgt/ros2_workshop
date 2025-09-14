#include "pointcloud_segmentation/segmentation_component.hpp"

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pointcloud_segmentation {

SegmentationComponent::SegmentationComponent(const rclcpp::NodeOptions &options)
    : rclcpp::Node("segmentation_component", options) {
  declareParameters();
  readParameters();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pub_ = this->create_publisher<Cloud>(params_.topics.output, rclcpp::QoS(10));

  // message_filters subscribers: pass Node* and rmw_qos_profile_t
  sub_a_ = std::make_shared<message_filters::Subscriber<Cloud>>(
      this, params_.topics.cloud_a, rmw_qos_profile_default);
  sub_b_ = std::make_shared<message_filters::Subscriber<Cloud>>(
      this, params_.topics.cloud_b, rmw_qos_profile_default);

  SyncPolicy policy(params_.sync.queue_size);
  policy.setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(params_.sync.slop));
  sync_ = std::make_shared<Sync>(policy);
  sync_->connectInput(*sub_a_, *sub_b_);
  sync_->registerCallback(std::bind(&SegmentationComponent::syncedCallback,
                                    this, std::placeholders::_1,
                                    std::placeholders::_2));

  RCLCPP_INFO(
      get_logger(),
      "Started. Subscribing to '%s' & '%s', publishing '%s'. Map frame: '%s'",
      params_.topics.cloud_a.c_str(), params_.topics.cloud_b.c_str(),
      params_.topics.output.c_str(), params_.frames.map.c_str());
}

void SegmentationComponent::declareParameters() {
  // Group: topics
  this->declare_parameter("topics.cloud_a", params_.topics.cloud_a);
  this->declare_parameter("topics.cloud_b", params_.topics.cloud_b);
  this->declare_parameter("topics.output", params_.topics.output);

  // Group: frames
  this->declare_parameter("frames.map", params_.frames.map);

  // Group: filters
  this->declare_parameter("filters.voxel_leaf", params_.filters.voxel_leaf);
  this->declare_parameter("filters.z.min", params_.filters.z.min);
  this->declare_parameter("filters.z.max", params_.filters.z.max);

  // Group: clustering
  this->declare_parameter("clustering.tolerance", params_.clustering.tolerance);
  this->declare_parameter("clustering.min_size", params_.clustering.min_size);
  this->declare_parameter("clustering.max_size", params_.clustering.max_size);

  // Group: sync
  this->declare_parameter("sync.queue_size", params_.sync.queue_size);
  this->declare_parameter("sync.slop", params_.sync.slop);
}

void SegmentationComponent::readParameters() {
  params_.use_sim_time = this->get_parameter("use_sim_time").as_bool();

  params_.topics.cloud_a = this->get_parameter("topics.cloud_a").as_string();
  params_.topics.cloud_b = this->get_parameter("topics.cloud_b").as_string();
  params_.topics.output = this->get_parameter("topics.output").as_string();

  params_.frames.map = this->get_parameter("frames.map").as_string();

  params_.filters.voxel_leaf =
      this->get_parameter("filters.voxel_leaf").as_double();
  params_.filters.z.min = this->get_parameter("filters.z.min").as_double();
  params_.filters.z.max = this->get_parameter("filters.z.max").as_double();

  params_.clustering.tolerance =
      this->get_parameter("clustering.tolerance").as_double();
  params_.clustering.min_size =
      this->get_parameter("clustering.min_size").as_int();
  params_.clustering.max_size =
      this->get_parameter("clustering.max_size").as_int();

  params_.sync.queue_size = this->get_parameter("sync.queue_size").as_int();
  params_.sync.slop = this->get_parameter("sync.slop").as_double();
}

bool SegmentationComponent::lookupAndTransform(const Cloud::ConstSharedPtr &in,
                                               Cloud &out) const {
  try {
    auto tf = tf_buffer_->lookupTransform(params_.frames.map,
                                          in->header.frame_id, in->header.stamp,
                                          rclcpp::Duration::from_seconds(0.2));
    tf2::doTransform(*in, out, tf);
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "TF transform failed %s -> %s: %s",
                in->header.frame_id.c_str(), params_.frames.map.c_str(),
                ex.what());
    return false;
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
SegmentationComponent::toPclXYZ(const Cloud &msg) const {
  auto cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(msg, *cloud);
  return cloud;
}

sensor_msgs::msg::PointCloud2::UniquePtr
SegmentationComponent::toRosMsg(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                const std_msgs::msg::Header &header) const {
  auto out = std::make_unique<Cloud>();
  pcl::toROSMsg(cloud, *out);
  out->header = header;
  return out;
}

void SegmentationComponent::syncedCallback(const Cloud::ConstSharedPtr &a,
                                           const Cloud::ConstSharedPtr &b) {
  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                       "Sync hit: stamps a=%.3f b=%.3f",
                       rclcpp::Time(a->header.stamp).seconds(),
                       rclcpp::Time(b->header.stamp).seconds());

  // 1) Transform both to map frame
  Cloud a_map, b_map;
  if (!lookupAndTransform(a, a_map) || !lookupAndTransform(b, b_map)) {
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                         "Skipping pair due to TF failure.");
    return;
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                       "Transformed to '%s' (a_pts=%u, b_pts=%u)",
                       params_.frames.map.c_str(), a_map.width * a_map.height,
                       b_map.width * b_map.height);

  // 2) Concatenate
  auto pa = toPclXYZ(a_map);
  auto pb = toPclXYZ(b_map);
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(
      new pcl::PointCloud<pcl::PointXYZ>());
  *merged = (*pa) + (*pb);
  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                       "Merged size = %zu", merged->size());

  // 3) Voxel downsampling
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(merged);
  voxel.setLeafSize(params_.filters.voxel_leaf, params_.filters.voxel_leaf,
                    params_.filters.voxel_leaf);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>());
  voxel.filter(*ds);
  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                       "Voxel (leaf=%.3f) -> %zu pts",
                       params_.filters.voxel_leaf, ds->size());

  // 4) Height (Z) filter
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(ds);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(params_.filters.z.min, params_.filters.z.max);
  pcl::PointCloud<pcl::PointXYZ>::Ptr zf(new pcl::PointCloud<pcl::PointXYZ>());
  pass.filter(*zf);
  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                       "Z filter [%.2f, %.2f] -> %zu pts",
                       params_.filters.z.min, params_.filters.z.max,
                       zf->size());

  if (zf->empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "No points after filtering, skipping clustering.");
    return;
  }

  // 5) Euclidean clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(zf);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(params_.clustering.tolerance);
  ec.setMinClusterSize(params_.clustering.min_size);
  ec.setMaxClusterSize(params_.clustering.max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(zf);
  ec.extract(cluster_indices);

  size_t clustered_pts = 0;
  for (const auto &inds : cluster_indices)
    clustered_pts += inds.indices.size();
  RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), 2000,
      "Clusters: %zu (clustered_pts=%zu, tol=%.2f, min=%d, max=%d)",
      cluster_indices.size(), clustered_pts, params_.clustering.tolerance,
      params_.clustering.min_size, params_.clustering.max_size);

  // 6) Colorize cluster-wise
  pcl::PointCloud<pcl::PointXYZRGB> colored;
  colored.header = zf->header;
  colored.reserve(zf->size());

  const std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> palette = {
      {255, 0, 0},   {0, 255, 0},   {0, 0, 255},   {255, 255, 0},
      {255, 0, 255}, {0, 255, 255}, {255, 128, 0}, {128, 0, 255},
      {0, 128, 255}, {128, 255, 0}, {255, 0, 128}, {0, 255, 128}};

  int cluster_id = 0;
  for (const auto &inds : cluster_indices) {
    auto [r, g, b] = palette[cluster_id % static_cast<int>(palette.size())];
    for (int idx : inds.indices) {
      const auto &p = zf->points[idx];
      pcl::PointXYZRGB q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;
      q.r = r;
      q.g = g;
      q.b = b;
      colored.points.push_back(q);
    }
    ++cluster_id;
  }
  colored.width = static_cast<uint32_t>(colored.points.size());
  colored.height = 1;
  colored.is_dense = false;

  // 7) Publish
  auto header = a_map.header; // already map frame; stamp from a
  header.frame_id = params_.frames.map;
  auto out = toRosMsg(colored, header);
  pub_->publish(std::move(out));

  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                       "Published %zu colored points in %zu clusters -> %s",
                       static_cast<size_t>(colored.points.size()),
                       cluster_indices.size(), params_.topics.output.c_str());
}

} // namespace pointcloud_segmentation
