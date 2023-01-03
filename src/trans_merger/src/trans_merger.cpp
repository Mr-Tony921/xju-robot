//
// Created by tony on 2023/1/3.
//

#include "trans_merger.h"

namespace xju::trans_merger {
auto dist(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2) -> double {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

auto dist2(float x, float y) -> float { return x * x + y * y; }

auto find_slope(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2) -> double {
  return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

void project(double sin_theta, double cos_theta, float x_src, float y_src, float& x_dst, float& y_dst) {
  x_dst = x_src * cos_theta - y_src * sin_theta;
  y_dst = y_src * cos_theta + x_src * sin_theta;
}

void angle_check(float& angle, float angle_min, float angle_max) {
  if (angle_min < -M_PI && angle > angle_min + DoublePI && angle < M_PI) {
    angle -= DoublePI;
  } else if (angle_max > M_PI && angle < angle_max - DoublePI && angle > -M_PI) {
    angle += DoublePI;
  }
}

TransMerger::TransMerger() : private_nh_("~") {
  private_nh_.param("cluster_dist", cluster_dist_, 0.5);
  private_nh_.param("check_range", check_range_, 4);

  // filter config, no merger process, used to filter sensor noises, avoid speedbump, etc.
  load_filter_params();

  // merger and filter config, used to merge several topics into one, and filter noises, avoid speedbump, etc.
  load_merger_params();

  // prepare output basic information
  output_.height = 1;
  output_.fields.resize(3);
  output_.fields[0].name = "x";
  output_.fields[0].offset = 0;
  output_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  output_.fields[1].name = "y";
  output_.fields[1].offset = 4;
  output_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  output_.fields[2].name = "z";
  output_.fields[2].offset = 8;
  output_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  output_.point_step = 12;
  output_.is_dense = true;

  subscriber_list_.reserve(20);
  tf_.reset(new tf2_ros::Buffer(ros::Duration(2)));
  tfl_.reset(new tf2_ros::TransformListener(*tf_));
}

TransMerger::~TransMerger() {
  for (auto& m : merger_list_) {
    m.timer.stop();
  }
}

void TransMerger::load_filter_params() {
  if (!private_nh_.hasParam("filter/topics")) {
    ROS_WARN("TransMerger : No filter info found!");
    return;
  }

  std::vector<std::string> filter_topics;
  private_nh_.getParam("filter/topics", filter_topics);
  for (auto& topic : filter_topics) {
    OutputParam output_param;
    // prepare output params
    output_param.name = topic;
    output_param.publisher = private_nh_.advertise<sensor_msgs::PointCloud2>(topic, 1);

    if (private_nh_.hasParam("filter/" + topic + "/output/clear_itself")) {
      std::vector<double> params;
      private_nh_.getParam("filter/" + topic + "/output/clear_itself", params);
      if (!output_param.clear_itself.set_value(params)) continue;
    }

    double filter_noise_projection_area_dist;
    output_param.frame = get_param(private_nh_, "filter" + topic + "/output/frame", std::string("self"));
    output_param.passby = get_param(private_nh_, "filter" + topic + "/output/passby", false);
    output_param.filter_noise_density_num =
      get_param(private_nh_, "filter" + topic + "/output/filter_noise_density_num", 0);
    filter_noise_projection_area_dist =
      get_param(private_nh_, "filter" + topic + "/output/filter_noise_projection_area_dist", 0.0);
    output_param.filter_noise_projection_area_dist2 = std::pow(filter_noise_projection_area_dist, 2);

    if (!output_param.passby) {
      std::vector<double> params;
      private_nh_.getParam("filter/" + topic + "/output/pcl_params", params);
      if (!output_param.params.set_value(params)) continue;
    }

    // if output as pointcloud, filter input pointcloud before output
    if (private_nh_.hasParam("filter/" + topic + "/input/pcl_topic")) {
      std::string subscribe_name;
      private_nh_.getParam("filter/" + topic + "/input/pcl_topic", subscribe_name);
      auto sub = private_nh_.subscribe<sensor_msgs::PointCloud2>(
        subscribe_name, 1,
        boost::bind(&TransMerger::pointcloud_callback, this, _1, output_param, -1, -1));
      subscriber_list_.emplace_back(sub);
    } else {
      // if output as laserscan, only do output process
      std::string subscribe_name;
      private_nh_.getParam("filter/" + topic + "/input/lds_topic", subscribe_name);
      auto sub = private_nh_.subscribe<sensor_msgs::LaserScan>(
        subscribe_name, 1,
        boost::bind(&TransMerger::laserscan_callback, this, _1, output_param, -1, -1));
      subscriber_list_.emplace_back(sub);
    }
  }
}

void TransMerger::load_merger_params() {
  if (!private_nh_.hasParam("merger/topics")) {
    ROS_WARN("TransMerger : No merger info found!");
    return;
  }
  std::vector<std::string> merger_topics;
  private_nh_.getParam("merger/topics", merger_topics);
  for (auto& topic : merger_topics) {
    OutputParam output_param;
    // prepare output params
    output_param.name = topic;
    output_param.publisher = private_nh_.advertise<sensor_msgs::PointCloud2>(topic, 1);

    if (private_nh_.hasParam("merger/" + topic + "/output/clear_itself")) {
      std::vector<double> params;
      private_nh_.getParam("merger/" + topic + "/output/clear_itself", params);
      if (!output_param.clear_itself.set_value(params)) continue;
    }

    double filter_noise_projection_area_dist;
    output_param.frame = get_param(private_nh_, "merger" + topic + "/output/frame", std::string("self"));
    output_param.passby = get_param(private_nh_, "merger" + topic + "/output/passby", false);
    output_param.filter_noise_density_num =
      get_param(private_nh_, "merger" + topic + "/output/filter_noise_density_num", 0);
    filter_noise_projection_area_dist =
      get_param(private_nh_, "merger" + topic + "/output/filter_noise_projection_area_dist", 0.0);

    output_param.filter_noise_projection_area_dist2 = std::pow(filter_noise_projection_area_dist, 2);

    // prepare merger params
    Merger merger;

    std::vector<double> params;
    private_nh_.getParam("merger/" + topic + "/output/pcl_params", params);
    if (!output_param.params.set_value(params)) continue;

    // prepare input params
    // indicating that it is a pointcloud topic as input
    if (private_nh_.hasParam("merger/" + topic + "/input/pcl_topics")) {
      std::vector<std::string> subscribe_names;
      private_nh_.getParam("merger/" + topic + "/input/pcl_topics", subscribe_names);
      for (auto& pcl_topic : subscribe_names) {
        auto sub = private_nh_.subscribe<sensor_msgs::PointCloud2>(
          pcl_topic, 1,
          boost::bind(&TransMerger::pointcloud_callback, this, _1, output_param, merger_list_.size(),
                      merger.clouds.size()));
        merger.clouds.emplace_back(new pcl::PCLPointCloud2(), false);
        subscriber_list_.emplace_back(sub);
      }
    }

    // indicating that it is a laserscan topic as input
    if (private_nh_.hasParam("merger/" + topic + "/input/lds_topics")) {
      std::vector<std::string> subscribe_names;
      private_nh_.getParam("merger/" + topic + "/input/lds_topics", subscribe_names);
      for (auto& lds_topic : subscribe_names) {
        auto sub = private_nh_.subscribe<sensor_msgs::LaserScan>(
          lds_topic, 1,
          boost::bind(&TransMerger::laserscan_callback, this, _1, output_param, merger_list_.size(),
                      merger.clouds.size()));
        merger.clouds.emplace_back(new pcl::PCLPointCloud2(), false);
        subscriber_list_.emplace_back(sub);
      }
    }
    merger.timer = private_nh_.createTimer(
      ros::Duration(params[7]),
      boost::bind(&TransMerger::timer_callback, this, _1, merger_list_.size(), output_param));
    merger_list_.emplace_back(merger);
  }
}

void TransMerger::laserscan_callback(const scan_ptr_t& scan, OutputParam& output_param, int merger_index,
                                     int merger_input_index) {
  // project scan to pointcloud with target frame
  projector_.transformLaserScanToPointCloud(output_param.frame, *scan, output_param.tmp_cloud, *tf_, -1, 1);

  if (merger_index >= 0) {
    auto& merger = merger_list_[static_cast<uint32_t>(merger_index)];
    // if cloud_data is available for new data
    boost::mutex::scoped_lock lock(merge_mutex_);

    auto& input = merger.clouds[static_cast<uint32_t>(merger_input_index)];

    pcl_conversions::toPCL(output_param.tmp_cloud, *(input.first));
    input.second = true;
    return;
  }

  publish_data(output_param);
}

void TransMerger::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud, OutputParam& output_param,
                                      int merger_index, int merger_input_index) {
  if (cloud->header.frame_id != output_param.frame) {
    if (!pcl_ros::transformPointCloud(output_param.frame, *cloud, output_param.tmp_cloud, *tf_)) {
      ROS_ERROR("TransMerger : cloud cb transform error");
      return;
    }
  } else {
    output_param.tmp_cloud = *cloud;
  }

  if (merger_index >= 0) {
    auto& merger = merger_list_[static_cast<uint32_t>(merger_index)];
    // if cloud_data is available for new data
    boost::mutex::scoped_lock lock(merge_mutex_);

    auto& input = merger.clouds[static_cast<uint32_t>(merger_input_index)];

    pcl_conversions::toPCL(output_param.tmp_cloud, *(input.first));
    input.second = true;
    return;
  }

  publish_data(output_param);
}

void TransMerger::timer_callback(const ros::TimerEvent& event, int merger_index, OutputParam& output_param) {
  auto& merger = merger_list_[static_cast<uint32_t>(merger_index)];
  pcl::PCLPointCloud2::Ptr merged_cloud = nullptr;
  {
    boost::mutex::scoped_lock lock(merge_mutex_);

    for (auto& input : merger.clouds) {
      if (!input.second) continue;

      if (merged_cloud == nullptr) {
        merged_cloud = input.first;
      } else {
        pcl::concatenatePointCloud(*merged_cloud, *input.first, *merged_cloud);
      }

      input.second = false;
    }
  }
  if (merged_cloud == nullptr) return;
  pcl_conversions::moveFromPCL(*merged_cloud, output_param.tmp_cloud);
  publish_data(output_param);
}

auto TransMerger::get_map_transformation(Eigen::Quaternionf& Qmt, Eigen::Vector3f& tmt) -> bool {
  geometry_msgs::TransformStamped Tmt_transform;
  if (tf_->canTransform("map", output_.header.frame_id, output_.header.stamp, ros::Duration(0.001))) {
    try {
      Tmt_transform =
        tf_->lookupTransform("map", output_.header.frame_id, output_.header.stamp, ros::Duration(0.2));
    } catch (tf2::TransformException& ex) {
      ROS_ERROR_STREAM(
        "TransMerger : " + output_.header.frame_id + " to map tf transform failed: " << ex.what());
      return false;
    }
  } else {
    try {
      Tmt_transform = tf_->lookupTransform("map", output_.header.frame_id, ros::Time(0), ros::Duration(0.2));
    } catch (tf2::TransformException& ex) {
      ROS_ERROR_STREAM(
        "TransMerger : " + output_.header.frame_id + " to map tf transform failed: " << ex.what());
      return false;
    }
  }

  Qmt.w() = Tmt_transform.transform.rotation.w;
  Qmt.x() = Tmt_transform.transform.rotation.x;
  Qmt.y() = Tmt_transform.transform.rotation.y;
  Qmt.z() = Tmt_transform.transform.rotation.z;

  tmt.x() = static_cast<float>(Tmt_transform.transform.translation.x);
  tmt.y() = static_cast<float>(Tmt_transform.transform.translation.y);
  tmt.z() = static_cast<float>(Tmt_transform.transform.translation.z);
  return true;
}

auto TransMerger::get_base_link_transformation(Eigen::Quaternionf& Qbt, Eigen::Vector3f& tbt) -> bool {
  geometry_msgs::TransformStamped Tbt_transform;
  try {
    Tbt_transform = tf_->lookupTransform("base_link", output_.header.frame_id, ros::Time(0), ros::Duration(0.2));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR_STREAM(
      "TransMerger : " + output_.header.frame_id + " to base_link tf transform failed: " << ex.what());
    return false;
  }

  Qbt.w() = Tbt_transform.transform.rotation.w;
  Qbt.x() = Tbt_transform.transform.rotation.x;
  Qbt.y() = Tbt_transform.transform.rotation.y;
  Qbt.z() = Tbt_transform.transform.rotation.z;

  tbt.x() = static_cast<float>(Tbt_transform.transform.translation.x);
  tbt.y() = static_cast<float>(Tbt_transform.transform.translation.y);
  tbt.z() = static_cast<float>(Tbt_transform.transform.translation.z);
  return true;
}

auto TransMerger::filter_noise_density(const OutputParam& output_param, std::vector<float>& ranges,
                                       std::vector<float> heights) -> uint32_t {
  uint32_t valid_pointcloud = 0;

  auto valid_check_range = output_param.params.range_max - 0.02;

  // prepare cosine_map
  auto cosine_map = get_cosine_map(output_param.params.angle_min, output_param.params.angle_max,
                                   output_param.params.angle_inc, output_.width);

  for (int i = 0; i < output_.width; ++i) {
    // invalid points to pointcloud2
    if (ranges[i] > output_param.params.range_max) continue;

    // noise points to pointcloud2
    int valid_count = 1;

    for (int j = -output_param.filter_noise_density_num * 2; j <= output_param.filter_noise_density_num * 2; ++j) {
      if (j == 0) continue;
      auto idx = i + j;
      if (idx < 0 || idx >= output_.width) continue;

      if (std::abs(ranges[idx] - ranges[i]) < cluster_dist_) {
        if (++valid_count >= output_param.filter_noise_density_num) break;
      }
    }
    if (valid_count < output_param.filter_noise_density_num) continue;

    if (ranges[i] > valid_check_range) {
      // density filter
      bool back_valid = false;

      for (int j = 1; j < check_range_; ++j) {
        int idx = i + j;
        if (idx >= output_.width) continue;
        if (ranges[idx] < valid_check_range) {
          back_valid = true;
          break;
        }
      }
      if (back_valid) continue;
    }

    // valid points to pointcloud2
    auto* point = (float*) &output_.data[valid_pointcloud++ * output_.point_step];
    point[0] = ranges[i] * cosine_map(i, 0);
    point[1] = ranges[i] * cosine_map(i, 1);
    point[2] = heights[i];
  }

  return valid_pointcloud;
}

void TransMerger::publish_data(OutputParam& output_param) {
  // update output header part
  output_.header.seq = output_param.tmp_cloud.header.seq;
  output_.header.stamp = output_param.tmp_cloud.header.stamp;
  output_.header.frame_id = output_param.frame;

  Eigen::Quaternionf Qmt;
  Eigen::Vector3f tmt;
  if (!get_map_transformation(Qmt, tmt)) return;

  // passby mode
  if (output_param.passby) {
    output_param.publisher.publish(output_param.tmp_cloud);
    return;
  }

  // update output other part
  auto angle_inc_inv = 1 / output_param.params.angle_inc;
  output_.width = std::ceil((output_param.params.angle_max - output_param.params.angle_min) * angle_inc_inv);
  output_.row_step = output_.point_step * output_.width;
  output_.data.clear();
  output_.data.resize(output_.row_step);

  // laserscan mode to calc
  auto heights = std::vector<float>(output_.width, 0);
  auto ranges = std::vector<float>(output_.width, output_param.params.range_max + 0.01);
  auto valid_range = output_param.params.range_max - 0.01;

  // clear itself function
  if (output_param.clear_itself.trust_angle_min != output_param.clear_itself.trust_angle_max) {
    auto trust_left = static_cast<size_t>(
      (output_param.clear_itself.trust_angle_max - output_param.params.angle_min) * angle_inc_inv);
    auto trust_right = static_cast<size_t>(
      (output_param.clear_itself.trust_angle_min - output_param.params.angle_min) * angle_inc_inv);
    std::fill(ranges.begin() + trust_right, ranges.begin() + trust_left, valid_range);
  }

  // obstacle_height_around_speedbump function parameters
  Eigen::Quaternionf Qbt;
  Eigen::Vector3f tbt;
  if (!get_base_link_transformation(Qbt, tbt)) return;

  auto range_min_sq = output_param.params.range_min * output_param.params.range_min;
  auto range_max_sq = output_param.params.range_max * output_param.params.range_max;

  // avoid pointcloud data incorrect
  sensor_msgs::PointCloud2Modifier modifier(output_param.tmp_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  // filter_noise_projection_area_dist function parameters
  auto filter_noise_projection_area_flag = output_param.filter_noise_projection_area_dist2 > 0;
  NoiseProjectionFilter filter_noiser(output_param.filter_noise_projection_area_dist2);

  auto iter = sensor_msgs::PointCloud2ConstIterator<float>(output_param.tmp_cloud, "x");

  for (; iter != iter.end(); ++iter) {
    if (std::isnan(iter[0]) || std::isnan(iter[1]) || std::isnan(iter[2])) continue;

    if (iter[2] < output_param.params.height_min || iter[2] > output_param.params.height_max) continue;

    float angle = std::atan2(iter[1], iter[0]);
    angle_check(angle, output_param.params.angle_min, output_param.params.angle_max);

    if (angle < output_param.params.angle_min || angle > output_param.params.angle_max) continue;

    auto range_sq = dist2(iter[0], iter[1]);
    if (range_sq < range_min_sq) continue;

    int index = (angle - output_param.params.angle_min) * angle_inc_inv;

    // filter_noise_projection_area_dist function
    if (filter_noise_projection_area_flag && filter_noiser.add_point(range_sq, iter, index)) continue;

    if (range_sq > range_max_sq) {
      ranges[index] = valid_range;
      heights[index] = iter[2];
      continue;
    }

    if (ranges[index] * ranges[index] < range_sq) continue;

    ranges[index] = std::sqrt(range_sq);
    heights[index] = iter[2];
  }

  // filter_noise_projection_area_dist function
  if (filter_noise_projection_area_flag) {
    filter_noiser.filter(ranges);
  }

  uint32_t valid_pointcloud = 0;

  // filter_noise_density_num function
  output_.width = filter_noise_density(output_param, ranges, heights);
  output_.row_step = output_.point_step * output_.width;
  output_.data.resize(output_.row_step);
  output_param.publisher.publish(output_);
}

auto TransMerger::get_cosine_map(float angle_min, float angle_max, float angle_inc, size_t size)
-> Eigen::ArrayXXd {
  std::stringstream s;
  s << angle_min << "," << angle_max << "," << angle_inc;
  auto it = cosine_map_.find(s.str());
  if (it != cosine_map_.end()) {
    return it->second;
  }

  auto cosine_map = Eigen::ArrayXXd(size, 2);

  // Spherical->Cartesian projection
  for (size_t i = 0; i < size; ++i) {
    cosine_map(i, 0) = std::cos(angle_min + (double) i * angle_inc);
    cosine_map(i, 1) = std::sin(angle_min + (double) i * angle_inc);
  }
  cosine_map_[s.str()] = cosine_map;
  return cosine_map;
}

auto NoiseProjectionFilter::add_point(float range, sensor_msgs::PointCloud2ConstIterator<float> iter, int index)
-> bool {
  if (range < filter_noise_projection_area_dist2_ && contour_points_.size() < FilterBuffer) {
    contour_points_.emplace_back(iter[1], iter[2]);
    contour_info_.emplace_back(iter[0], iter[1], iter[2], index, range);
    return true;
  }
  return false;
}

void NoiseProjectionFilter::filter(std::vector<float>& ranges) {
  auto contour_area = double{-1.0};
  if (contour_points_.size() > 3 && contour_points_.size() < FilterBuffer) {
    contour_area = cv::contourArea(contour_points_);
  }

  if (contour_area > MinAreaOfObstacle || contour_area < 0) {
    for (const auto& p : contour_info_) {
      ranges[p.index] = std::min(ranges[p.index], static_cast<float>(std::sqrt(p.range_sq)));
    }
  }
}
}  // namespace xju::trans_merger
