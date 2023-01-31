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

void XjuBridge::timer_callback(const ros::TimerEvent& e) {
  fusion_analysis_pub_.publish(pub_msg_);
}
}
