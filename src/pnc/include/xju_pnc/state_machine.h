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

#include "mbf_msgs/MoveBaseAction.h"
#include "mbf_msgs/ExePathAction.h"
#include "xju_pnc/exe_path.h"
#include "xju_pnc/record_start.h"
#include "xju_pnc/record_stop.h"

namespace xju::pnc {
constexpr static const char* NODE_NAME = "xju_state_machine";
constexpr static const char* DEFAULT_DIR = "/home/tony/course_ws/path";
constexpr static const char* RECORD_START_SRV = "xju_record_start";
constexpr static const char* RECORD_STOP_SRV = "xju_record_stop";
constexpr static const char* EXE_PATH_SRV = "xju_task";
constexpr static const double DEG2RAD = M_PI / 180;
constexpr static const double RECORD_PATH_LEN_DENS = 0.05;
constexpr static const double RECORD_PATH_AGU_DENS = 10 * DEG2RAD;

using GotoCtrl = actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>;
using ExeCtrl = actionlib::SimpleActionClient<mbf_msgs::ExePathAction>;

enum class StateValue : uint8_t {
  Idle = 0,
  Record,
  Run,
  Pause
};

class StateMachine {
public:
  StateMachine();

  ~StateMachine();

  void init();

  void run();

  void stop();

private:
  void pub_zero_vel();

  void cancel_goal();

  void reset();

  auto robot_pose() -> std::optional<geometry_msgs::Pose>;

  auto distance(geometry_msgs::Pose const& a, geometry_msgs::Pose const& b) -> std::pair<double, double>;

  auto nearest_index(nav_msgs::Path const& path,
                     size_t const& index,
                     int lb = std::numeric_limits<int>::max(),
                     int rb = std::numeric_limits<int>::max()) -> size_t;

  auto send_goto(size_t const& index) -> bool;

  void goto_done(const actionlib::SimpleClientGoalState& state, const mbf_msgs::MoveBaseResultConstPtr& result);

  auto send_exe(size_t const& index) -> bool;

  void exe_done(const actionlib::SimpleClientGoalState& state, const mbf_msgs::ExePathResultConstPtr& result);

  auto exe_path_service(xju_pnc::exe_path::Request& req, xju_pnc::exe_path::Response& resp) -> bool;

  auto record_start_service(xju_pnc::record_start::Request& req, xju_pnc::record_start::Response& resp) -> bool;

  auto record_stop_service(xju_pnc::record_stop::Request& req, xju_pnc::record_stop::Response& resp) -> bool;

  void record_path();

  void visualize_poses(std::vector<geometry_msgs::Pose> const& poses, ros::Publisher const& pub);

  auto check_file_exist(std::string& file_path) -> bool;

  auto write_file(std::string& file_path) -> bool;

  auto read_file(std::string& file_path) -> bool;

private:
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tfl_;

  ros::Publisher vel_pub_;
  ros::Publisher record_path_pub_;
  ros::ServiceServer record_start_srv_;
  ros::ServiceServer record_stop_srv_;
  ros::ServiceServer exe_path_srv_;

  std::unique_ptr<GotoCtrl> goto_ctrl_;
  std::unique_ptr<ExeCtrl> exe_ctrl_;

  std::string file_path_;
  std::vector<geometry_msgs::Pose> record_path_;

  StateValue cur_state_;
  size_t cur_index_;
  nav_msgs::Path cur_route_;
};
}
