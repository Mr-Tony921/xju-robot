//
// Created by tony on 2022/9/22.
//

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include "xju_simu/fusion_analysis.h"

namespace xju::simu {
constexpr static const char* NODE_NAME = "xju_bridge";
constexpr static const char* CommandTopic = "/cmd_vel";
constexpr static const char* FeedbackTopic = "/odom";
constexpr static const char* FusionTopic = "/fusion_analysis";
constexpr static const double WheelSeparation = 0.35;
constexpr static const double WheelRadius = 0.07;
constexpr static const double TimerDuration = 0.2;

class XjuBridge {
  typedef xju_simu::fusion_analysis fu_msg;
public:
  XjuBridge() = default;

  ~XjuBridge();

  void init();

private:
  void control_callback(const geometry_msgs::Twist::ConstPtr& msg);

  void feedback_callback(const nav_msgs::Odometry::ConstPtr& msg);

  void timer_callback(const ros::TimerEvent& e);

private:
  ros::Publisher fusion_analysis_pub_;

  ros::Subscriber control_sub_;
  ros::Subscriber feedback_sub_;

  ros::Timer fusion_analysis_timer_;

  fu_msg pub_msg_;
};
}
