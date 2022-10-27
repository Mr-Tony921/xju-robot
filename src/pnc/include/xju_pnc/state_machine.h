//
// Created by tony on 2022/10/11.
//

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "coverage_path_planner/GetPathInZone.h"
#include "mbf_msgs/MoveBaseAction.h"
#include "mbf_msgs/ExePathAction.h"
#include "xju_pnc/xju_task.h"

namespace xju::pnc {
constexpr static const char* NODE_NAME = "xju_state_machine";
constexpr static const char* DEFAULT_DIR = "/home/tony/course_ws/path";
constexpr static const char* RVIZ_POINT = "/clicked_point";
constexpr static const char* COSTMAP = "/move_base_flex/global_costmap/costmap";
constexpr static const char* COSTMAP_UPDATE = "/move_base_flex/global_costmap/costmap_updates";
constexpr static const char* TASK_SRV = "xju_task";
constexpr static const double DEG2RAD = M_PI / 180;
constexpr static const double RECORD_PATH_LEN_DENS = 0.05;
constexpr static const double RECORD_PATH_AGU_DENS = 10 * DEG2RAD;
constexpr static const int WAIT_COUNT = 5 * 10; // 5s
constexpr static const int PATH_SAFE_DIS_NUM = 1.3 / RECORD_PATH_LEN_DENS; // 1.3m

using GotoCtrl = actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>;
using ExeCtrl = actionlib::SimpleActionClient<mbf_msgs::ExePathAction>;

enum class StateValue : uint8_t {
  Idle = 0,
  Record,
  Run,
  Pause
};

enum class RunStateValue : uint8_t {
  Goto = 0,
  Follow,
  Wait
};

class StateMachine {
public:
  StateMachine();

  ~StateMachine();

  void init();

  void run();

  void stop();

private:
  void running_state();

  void pub_zero_vel();

  void cancel_goal();

  void reset();

  auto robot_pose() -> std::optional<geometry_msgs::Pose>;

  auto distance(geometry_msgs::Pose const& a, geometry_msgs::Pose const& b) -> std::pair<double, double>;

  auto nearest_info(nav_msgs::Path const& path,
                    size_t const& index,
                    int lb = std::numeric_limits<int>::max(),
                    int rb = std::numeric_limits<int>::max(),
                    bool following = false) -> std::pair<size_t, double>;

  void costmap_cb(nav_msgs::OccupancyGrid::ConstPtr const& msg);

  void costmap_update_cb(map_msgs::OccupancyGridUpdate::ConstPtr const& msg);

  void point_cb(geometry_msgs::PointStampedConstPtr const& msg);

  auto is_free(const geometry_msgs::PoseStamped& pose) const -> bool;

  auto is_danger(const geometry_msgs::PoseStamped& pose) const -> bool;

  auto pose_cost(const geometry_msgs::PoseStamped& pose) const -> uint8_t;

  auto send_goto(size_t const& index) -> bool;

  void goto_done(const actionlib::SimpleClientGoalState& state, const mbf_msgs::MoveBaseResultConstPtr& result);

  auto send_exe(size_t const& index) -> bool;

  void exe_done(const actionlib::SimpleClientGoalState& state, const mbf_msgs::ExePathResultConstPtr& result);

  auto task_service(xju_pnc::xju_task::Request& req, xju_pnc::xju_task::Response& resp) -> bool;

  void record_path();

  auto check_file_exist(std::string& file_path) -> bool;

  auto write_file(std::string& file_path, std::vector<nav_msgs::Path> const& paths) -> bool;

  auto read_file(std::string& file_path) -> bool;

private:
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tfl_;

  ros::Publisher vel_pub_;
  ros::Publisher record_path_pub_;
  ros::Publisher cur_pose_pub_;
  ros::Publisher record_point_pub_;
  ros::Subscriber point_sub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_update_sub_;
  ros::ServiceServer task_srv_;
  ros::ServiceClient coverage_srv_;

  std::unique_ptr<GotoCtrl> goto_ctrl_;
  std::unique_ptr<ExeCtrl> exe_ctrl_;

  std::string file_path_;
  nav_msgs::Path record_path_;
  std::vector<geometry_msgs::PointStamped> record_points_;

  StateValue cur_state_;
  RunStateValue running_state_;
  size_t cur_index_;
  size_t obs_index_;
  nav_msgs::Path cur_route_;
  std::list<nav_msgs::Path> routes_;

  std::mutex map_update_mutex_;

  std::shared_ptr<costmap_2d::Costmap2D> costmap_;
  static uint8_t* cost_translation_;
};
}
