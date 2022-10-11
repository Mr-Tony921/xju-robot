//
// Created by tony on 2022/10/11.
//

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "xju_pnc/record_start.h"
#include "xju_pnc/record_stop.h"

namespace xju::pnc {
constexpr static const char* NODE_NAME = "xju_state_machine";
constexpr static const char* RECORD_START_SRV = "xju_record_start";
constexpr static const char* RECORD_STOP_SRV = "xju_record_stop";
constexpr static const double RECORD_PATH_LEN_DENS = 0.05;
constexpr static const double RECORD_PATH_AGU_DENS = 10 * M_PI / 180;

class StateMachine {
public:
  StateMachine();

  ~StateMachine();

  void init();

  void run();

  void stop();

private:
  void pub_zero_vel();

  auto robot_pose() -> std::optional<geometry_msgs::Pose>;

  auto record_start_service(xju_pnc::record_start::Request& req, xju_pnc::record_start::Response& resp) -> bool;

  auto record_stop_service(xju_pnc::record_stop::Request& req, xju_pnc::record_stop::Response& resp) -> bool;

  void record_path();

  void visualize_poses(std::vector<geometry_msgs::Pose> const& poses, ros::Publisher const& pub);

  auto check_file_exist(std::string& file_path) -> bool;

  auto write_file(std::string& file_path) -> bool;

private:
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tfl_;

  ros::Publisher vel_pub_;
  ros::Publisher record_path_pub_;
  ros::ServiceServer record_start_srv_;
  ros::ServiceServer record_stop_srv_;

  std::string file_path_;
  std::atomic_bool recording_;

  std::vector<geometry_msgs::Pose> record_path_;
};
}
