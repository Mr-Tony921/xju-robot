//
// Created by tony on 2022/9/22.
//

#include "xju_bridge.h"

namespace xju::simu {
XjuBridge::~XjuBridge() {
  fusion_analysis_timer_.stop();
}

void XjuBridge::init() {
  ros::NodeHandle nh;
  fusion_analysis_pub_ = nh.advertise<fu_msg>(FusionTopic, 10);
  odom_transform_pub_ = nh.advertise<geometry_msgs::TransformStamped>(OdomTransform, 10);
  control_sub_ = nh.subscribe(CommandTopic, 1, &XjuBridge::control_callback, this);
  feedback_sub_ = nh.subscribe(FeedbackTopic, 1, &XjuBridge::feedback_callback, this, ros::TransportHints().tcpNoDelay());
  fusion_analysis_timer_ = nh.createTimer(ros::Duration(TimerDuration), &XjuBridge::timer_callback, this);

  bool dense_cloud = false;
  ros::NodeHandle pnh("~");
  pnh.getParam("pub_density_cloud", dense_cloud);
  if (!dense_cloud) return;
  dense_pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>(DenPCLTopic, 1);
  ori_pcl_sub_ = nh.subscribe(OriPCLTopic, 1, &XjuBridge::ori_pcl_callback, this);
}

void XjuBridge::control_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  pub_msg_.linear_control = msg->linear.x;
  pub_msg_.angular_control = msg->angular.z;
  pub_msg_.lwheel_control = (msg->linear.x - 0.5 * msg->angular.z * WheelSeparation) / WheelRadius;
  pub_msg_.rwheel_control = (msg->linear.x + 0.5 * msg->angular.z * WheelSeparation) / WheelRadius;
}

void XjuBridge::feedback_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  static double last_lwheel_feedback = 0;
  static double last_rwheel_feedback = 0;
  pub_msg_.linear_feedback = msg->twist.twist.linear.x;
  pub_msg_.angular_feedback = msg->twist.twist.angular.z;
  pub_msg_.lwheel_feedback = (msg->twist.twist.linear.x - 0.5 * msg->twist.twist.angular.z * WheelSeparation) / WheelRadius;
  pub_msg_.rwheel_feedback = (msg->twist.twist.linear.x + 0.5 * msg->twist.twist.angular.z * WheelSeparation) / WheelRadius;
  pub_msg_.lwheel_acc = (pub_msg_.lwheel_feedback - last_lwheel_feedback) / TimerDuration;
  pub_msg_.rwheel_acc = (pub_msg_.rwheel_feedback - last_rwheel_feedback) / TimerDuration;
  last_lwheel_feedback = pub_msg_.lwheel_feedback;
  last_rwheel_feedback = pub_msg_.rwheel_feedback;
  pub_msg_.odom_pose.x = msg->pose.pose.position.x;
  pub_msg_.odom_pose.y = msg->pose.pose.position.y;
  pub_msg_.odom_pose.yaw = tf2::getYaw(msg->pose.pose.orientation);
  ot_msg_.header = msg->header;
  ot_msg_.child_frame_id = "base_link";
  ot_msg_.transform.translation.x = msg->pose.pose.position.x;
  ot_msg_.transform.translation.y = msg->pose.pose.position.y;
  ot_msg_.transform.translation.z = msg->pose.pose.position.z;
  ot_msg_.transform.rotation = msg->pose.pose.orientation;
  odom_transform_pub_.publish(ot_msg_);
}

/// we are based on dense cloud (no nan) in xju gazebo_plugins branch!!!
void XjuBridge::ori_pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  using namespace std;
  struct point {
  float x;
  float y;
  float z;
  float intensity;
  short ring = -1;
  float time;

  void setValue(float xx, float yy, float zz, float ii, short rr, float tt) {
    x = xx;
    y = yy;
    z = zz;
    intensity = ii;
    ring = rr;
    time = tt;
  };

  bool isVaild() {
    return ring >= 0;
  };
  };

  sensor_msgs::PointCloud2 den_cloud;
  den_cloud.header = msg->header;
  den_cloud.height = msg->height; // 1
  den_cloud.fields = msg->fields; // 6
  den_cloud.is_bigendian = msg->is_bigendian;
  den_cloud.point_step = msg->point_step;
  den_cloud.is_dense = msg->is_dense; // true, based on dense point cloud (no nan)
  den_cloud.data.resize(msg->data.size() * PointCloutDenseRatio); // max size
  uint8_t *ptr = den_cloud.data.data();

  auto update_msg_data = [&](uint8_t* data_ptr, point const& pt) {
    *((float*) (data_ptr + 0)) = pt.x;
    *((float*) (data_ptr + 4)) = pt.y;
    *((float*) (data_ptr + 8)) = pt.z;
    *((float*) (data_ptr + 12)) = pt.intensity;
    *((uint16_t*) (data_ptr + 16)) = pt.ring;
    *((float*) (data_ptr + 18)) = pt.time;
    data_ptr += msg->point_step;
    return data_ptr;
  };

  auto dense_cloud = [&](uint8_t* data_ptr, point& base, point const& target) {
    for (int i = 1; i < PointCloutDenseRatio; ++i) {
      auto coeff = static_cast<float>(i) / PointCloutDenseRatio;
      if (abs(target.x - base.x) > PointCloutNoDenseGap
          || abs(target.y - base.y) > PointCloutNoDenseGap
          || abs(target.z - base.z) > PointCloutNoDenseGap)
        continue;
      base.x = base.x + coeff * (target.x - base.x);
      base.y = base.y + coeff * (target.y - base.y);
      base.z = base.z + coeff * (target.z - base.z);
      base.intensity = base.intensity + coeff * (target.x - base.intensity);

      data_ptr = update_msg_data(data_ptr, base);
    }
    return data_ptr;
  };

  std::vector<point> first_vertical(16), current_vertical(16), last_vertical(16);
  auto iter_x = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "x");
  auto iter_y = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "y");
  auto iter_z = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "z");
  auto iter_intensity = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "intensity");
  auto iter_ring = sensor_msgs::PointCloud2ConstIterator<short>(*msg, "ring");
  auto iter_time = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "time");
  point p{};
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_ring, ++iter_time) {
    p.setValue(*iter_x, *iter_y, *iter_z, *iter_intensity, *iter_ring, *iter_time);
    current_vertical[*iter_ring] = p;
    if (last_vertical[*iter_ring].isVaild()) {
      ptr = dense_cloud(ptr, last_vertical[*iter_ring], p);
    } else { // there is no point before in this ring number
      first_vertical[*iter_ring] = p;
    }
    last_vertical[*iter_ring] = p;
    ptr = update_msg_data(ptr, p);
  }
  for (auto i = 0; i < 16; ++i) {
    if (first_vertical[i].isVaild() && last_vertical[i].isVaild()) {
      ptr = dense_cloud(ptr, last_vertical[i], first_vertical[i]);
    }
  }

  den_cloud.data.resize(ptr - den_cloud.data.data());
  den_cloud.width = den_cloud.data.size() / msg->point_step;
  den_cloud.row_step = den_cloud.data.size();

  dense_pcl_pub_.publish(den_cloud);
}

void XjuBridge::timer_callback(const ros::TimerEvent& e) {
  fusion_analysis_pub_.publish(pub_msg_);
}
}
