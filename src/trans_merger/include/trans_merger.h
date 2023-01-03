//
// Created by tony on 2023/1/3.
//

#pragma once

#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>
#include <cstring>
// ros related
#include <geometry_msgs/Point.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/GetMap.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
// tf2 related
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// sensor_msgs related
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
// pcl related
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
// opencv related
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace xju::trans_merger {
static const auto FilterBuffer = int32_t{30};
static const auto MinAreaOfObstacle = double{0.009};
static const auto DoublePI = double{2 * M_PI};

template<typename T>
auto get_param(ros::NodeHandle& nh, std::string const& name, T default_value) -> T {
  T value = default_value;

  if (nh.getParam(name, value)) {
    ROS_INFO_STREAM("Load parameter " + name + " success: " << value);
    return value;
  }

  ROS_INFO_STREAM("Load parameter " + name + " failed, default: " << value);

  return default_value;
}

struct PclParam {
double height_min;
double height_max;
double angle_min;
double angle_max;
double range_min;
double range_max;
double angle_inc;
double scan_time;

auto set_value(std::vector<double> params) -> bool {
  if (params.size() < 8) {
    ROS_ERROR_STREAM("TransMerger: pcl params length error!");
    return false;
  }
  height_min = params[0];
  height_max = params[1];
  angle_min = params[2];
  angle_max = params[3];
  range_min = params[4];
  range_max = params[5];
  angle_inc = params[6];
  scan_time = params[7];
  return true;
}
};

struct TrustedAngle {
double trust_angle_min = 0.0;
double trust_angle_max = 0.0;

auto set_value(std::vector<double> params) -> bool {
  if (params.size() < 2) {
    ROS_ERROR_STREAM("TransMerger: trusted angle params length error!");
    return false;
  }
  trust_angle_min = params[0];
  trust_angle_max = params[1];
  return true;
}
};

struct OutputParam {
std::string name;
bool passby;
int filter_noise_density_num;
double filter_noise_projection_area_dist2;

sensor_msgs::PointCloud2 tmp_cloud;

PclParam params;
std::string frame;
ros::Publisher publisher;
TrustedAngle clear_itself;
};

struct Merger {
ros::Timer timer;
std::vector<std::pair<pcl::PCLPointCloud2::Ptr, bool>> clouds;
};

struct ContourInfo {
ContourInfo(float a, float b, float c, int d, float e) : x(a), y(b), z(c), index(d), range_sq(e) {};
float x;
float y;
float z;
int index;
float range_sq;
};

class NoiseProjectionFilter {
public:
  NoiseProjectionFilter(double dist2) : filter_noise_projection_area_dist2_(dist2) {
    contour_info_.reserve(FilterBuffer);
    contour_points_.reserve(FilterBuffer);
  };

  ~NoiseProjectionFilter() = default;

  auto add_point(float range, sensor_msgs::PointCloud2ConstIterator<float> iter, int index) -> bool;

  void filter(std::vector<float>& ranges);

private:
  std::vector<ContourInfo> contour_info_;
  std::vector<cv::Point2f> contour_points_;
  double filter_noise_projection_area_dist2_;
};

class TransMerger {
public:
  using size_t = uint32_t;
  using zone_t = std::vector<cv::Point2f>;
  using scan_ptr_t = sensor_msgs::LaserScan::ConstPtr;

public:
  TransMerger();

  ~TransMerger();

  void timer_callback(const ros::TimerEvent& event, int merger_index, OutputParam& output_param);

  void laserscan_callback(const scan_ptr_t& scan, OutputParam& output_param, int merger_index = -1,
                          int merger_input_index = -1);

  void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud, OutputParam& output_param,
                           int merger_index = -1, int merger_input_index = -1);

private:
  void load_filter_params();

  void load_merger_params();

  auto get_map_transformation(Eigen::Quaternionf& Qmt, Eigen::Vector3f& tmt) -> bool;

  auto get_base_link_transformation(Eigen::Quaternionf& Qbt, Eigen::Vector3f& tbt) -> bool;

  auto filter_noise_density(const OutputParam& output_param, std::vector<float>& ranges, std::vector<float> heights)
  -> uint32_t;

  void publish_data(OutputParam& output_param);

  auto get_cosine_map(float angle_min, float angle_max, float angle_inc, size_t size) -> Eigen::ArrayXXd;

private:
  boost::mutex merge_mutex_;

  ros::NodeHandle private_nh_;

  laser_geometry::LaserProjection projector_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  // filter noise nearby
  double cluster_dist_;

  // output variable
  int check_range_;
  sensor_msgs::PointCloud2 output_;
  std::map<std::string, Eigen::ArrayXXd> cosine_map_;

  // timer list
  std::vector<Merger> merger_list_;
  std::vector<ros::Subscriber> subscriber_list_;
};
}  // namespace xju::trans_merger