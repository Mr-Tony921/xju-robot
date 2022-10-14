//
// Created by tony on 2022/10/11.
//

#include <chrono>
#include <fstream>
#include "state_machine.h"

namespace xju::pnc {
StateMachine::StateMachine()
  : tf_(new tf2_ros::Buffer()),
    tfl_(new tf2_ros::TransformListener(*tf_)),
    goto_ctrl_(new GotoCtrl("move_base_flex/move_base")),
    exe_ctrl_(new ExeCtrl("move_base_flex/exe_path")),
    cur_state_(StateValue::Idle),
    cur_index_(0) {
}

StateMachine::~StateMachine() {
  stop();
}

void StateMachine::init() {
  ros::NodeHandle nh;
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  record_path_pub_ = nh.advertise<nav_msgs::Path>("record_path", 1);
  record_start_srv_ = nh.advertiseService(RECORD_START_SRV, &StateMachine::record_start_service, this);
  record_stop_srv_ = nh.advertiseService(RECORD_STOP_SRV, &StateMachine::record_stop_service, this);
  exe_path_srv_ = nh.advertiseService(EXE_PATH_SRV, &StateMachine::exe_path_service, this);
  ROS_ERROR_COND(!goto_ctrl_->waitForServer(ros::Duration(5.0)), "move base action not online!");
  ROS_ERROR_COND(!exe_ctrl_->waitForServer(ros::Duration(5.0)), "exe path action not online!");
  ROS_INFO("Xju state machine initialized!");
}

void StateMachine::run() {
  using goalState = actionlib::SimpleClientGoalState;
  ros::Rate r(ros::Duration(0.1));
  while(ros::ok()) {
    ros::spinOnce();
    switch (cur_state_) {
      case StateValue::Idle:
        // do nothing, cancel goal and reset value in event
      case StateValue::Pause:
        // do nothing, cancel goal in event
        break;
      case StateValue::Record:
        record_path();
        visualize_poses(record_path_, record_path_pub_);
        break;
      case StateValue::Run:
        if (exe_ctrl_->getState() == goalState::SUCCEEDED
            && cur_route_.poses.size() - cur_index_ < 10
            && distance(robot_pose().value(), cur_route_.poses.back().pose).first < 0.5) {
          ROS_INFO("Exe path success");
          reset();
          break;
        }

        if (exe_ctrl_->getState() != goalState::ACTIVE) {
          if (goto_ctrl_->getState() == goalState::ACTIVE) break;
          auto err = distance(robot_pose().value(), cur_route_.poses.at(cur_index_).pose);
          if (err.first > 0.5 || err.second > 30 * DEG2RAD) {
            send_goto(cur_index_);
          } else {
            send_exe(cur_index_);
          }

          break;
        }

        cur_index_ = nearest_index(cur_route_, cur_index_, 5, 10);
        break;
    }

    r.sleep();
  }
}

void StateMachine::stop() {
  cancel_goal();
  // pub zero velocity for 1s
  for (auto i = 0; i < 10; ++i) {
    pub_zero_vel();
    ros::Duration(0.1).sleep();
  }
}

void StateMachine::pub_zero_vel() {
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.angular.z = 0;
  vel_pub_.publish(vel);
}

void StateMachine::cancel_goal() {
  if (!goto_ctrl_->getState().isDone()) {
    ROS_INFO("Cancel current goto goal");
    goto_ctrl_->cancelGoal();
  }

  if (!exe_ctrl_->getState().isDone()) {
    ROS_INFO("Cancel current exe path goal");
    exe_ctrl_->cancelGoal();
  }
}

void StateMachine::reset() {
  cur_state_ = StateValue::Idle;
  cur_index_ = 0;
  cur_route_.poses.clear();
  stop();
}

auto StateMachine::robot_pose() -> std::optional<geometry_msgs::Pose> {
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tf_->lookupTransform("map", "base_link", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("tf error: %s", ex.what());
    return std::nullopt;
  }

  geometry_msgs::Pose result;
  result.position.x = transformStamped.transform.translation.x;
  result.position.y = transformStamped.transform.translation.y;
  result.orientation = transformStamped.transform.rotation;
  return std::make_optional(result);
}

auto StateMachine::distance(geometry_msgs::Pose const& a,
                            geometry_msgs::Pose const& b) -> std::pair<double, double> {
  std::pair<double, double> result;
  result.first = std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
  result.second = std::abs(tf2::getYaw(a.orientation) - tf2::getYaw(b.orientation));
  return result;
}

auto StateMachine::nearest_index(nav_msgs::Path const& path, size_t const& index, int lb, int rb) -> size_t {
  int result = index;
  auto robot_pos = robot_pose();
  if (!robot_pos) return result;

  auto min_dis = std::numeric_limits<double>::max();
  lb = std::max(static_cast<int>(index) - lb, 0);
  rb = std::min(index + rb, path.poses.size());
  for (auto i = lb; i < rb; ++i) {
    auto err = distance(robot_pos.value(), path.poses[i].pose);
    if (err.first < min_dis &&  err.second < 30 * DEG2RAD) {
      min_dis = err.first;
      result = static_cast<size_t>(i);
    }
  }

  return result;
}

auto StateMachine::send_goto(size_t const& index) -> bool {
  if (index == std::numeric_limits<size_t>::max() || index > cur_route_.poses.size()) {
    ROS_ERROR_STREAM("send_goto index error " << index << " / " << cur_route_.poses.size());
    return false;
  }

  mbf_msgs::MoveBaseGoal goal{};
  goal.target_pose = cur_route_.poses.at(index);
  ROS_INFO("send_goto goal");
  goto_ctrl_->sendGoal(goal,
                       boost::bind(&StateMachine::goto_done, this, _1, _2),
                       GotoCtrl::SimpleActiveCallback(),
                       GotoCtrl::SimpleFeedbackCallback());
  return true;
}

void StateMachine::goto_done(const actionlib::SimpleClientGoalState& state,
                             const mbf_msgs::MoveBaseResultConstPtr& result) {
  ROS_INFO("MoveBase got state [%s]", state.toString().c_str());

  if (!result) return;
  ROS_INFO("MoveBase got result [%d]", result->outcome);
//  send_exe(current_index_);
}

auto StateMachine::send_exe(size_t const& index) -> bool {
  if (index == std::numeric_limits<size_t>::max() || index > cur_route_.poses.size()) {
    ROS_ERROR_STREAM("send_exe index error " << index << " / " << cur_route_.poses.size());
    return false;
  }

  mbf_msgs::ExePathGoal goal{};
  goal.path = cur_route_;
  cur_route_.poses.erase(cur_route_.poses.begin(), cur_route_.poses.begin() + cur_index_);
  ROS_INFO("send_exe goal");
  exe_ctrl_->sendGoal(goal,
                      boost::bind(&StateMachine::exe_done, this, _1, _2),
                      ExeCtrl::SimpleActiveCallback(),
                      ExeCtrl::SimpleFeedbackCallback());
  return true;
}

void StateMachine::exe_done(const actionlib::SimpleClientGoalState& state,
                            const mbf_msgs::ExePathResultConstPtr& result) {
  ROS_INFO("ExePath got state [%s]", state.toString().c_str());

  if (!result) return;
  ROS_INFO("ExePath got result [%d]", result->outcome);
}

auto StateMachine::exe_path_service(xju_pnc::exe_path::Request& req, xju_pnc::exe_path::Response& resp) -> bool {
  if (cur_state_ == StateValue::Record) {
    resp.message = "Recording path now, ignore exe_path_service.";
    return true; // return false will miss message in response
  }

  if (req.command == xju_pnc::exe_path::Request::PAUSE) {
    if (cur_state_ != StateValue::Run) {
      resp.message = "Not in RUNNING status, ignore pause command.";
    } else {
      cur_state_ = StateValue::Pause;
      stop();
      resp.message = "Task pause.";
    }
    return true;
  }

  if (req.command == xju_pnc::exe_path::Request::STOP) {
    if (cur_state_ != StateValue::Run && cur_state_ != StateValue::Pause) {
      resp.message = "Not in RUNNING and PAUSE status, ignore stop command.";
    } else {
      reset();
      resp.message = "Task stop.";
    }
    return true;
  }

  if (req.command == xju_pnc::exe_path::Request::START) {
    if (cur_state_ == StateValue::Pause) {
      resp.message = "Task resume.";
      cur_index_ = nearest_index(cur_route_, cur_index_, 0, 20);
      cur_state_ = StateValue::Run;
      return true;
    }

    reset();
    auto dir = req.dir;
    if (dir == "") dir = DEFAULT_DIR;
    file_path_ = dir + "/" +req.path_name;
    if (!check_file_exist(file_path_)) {
      resp.message = file_path_ + " not exist, ignore start command.";
      return true;
    }

    if (!read_file(file_path_)) {
      resp.message = "Read path in file failed.";
      return true;
    }

    cur_state_ = StateValue::Run;
    resp.message = "Start new task!";
    return true;
  }

  resp.message = "Wrong exe service command, do nothing.";
  return true;
}

auto
StateMachine::record_start_service(xju_pnc::record_start::Request& req, xju_pnc::record_start::Response& resp) -> bool {
  if (cur_state_ != StateValue::Idle) {
    resp.message = "Not in IDLE status, ignore record command.";
    return true;
  }

  auto name = req.path_name;
  if (name == "") {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << "teach_path_" << std::put_time(std::localtime(&t), "%F %T");
    name = ss.str();
  }

  auto dir = req.dir;
  if (dir == "") {
    dir = DEFAULT_DIR;
  }

  file_path_ = dir + "/" + name;
  if (check_file_exist(file_path_)) {
    resp.message = "File already exist, ignore this operation.";
    return true;
  }

  record_path_.clear();
  cur_state_ = StateValue::Record;
  resp.message = "Start recording path, save to " + file_path_ + ".";
  return true;
}

auto
StateMachine::record_stop_service(xju_pnc::record_stop::Request& req, xju_pnc::record_stop::Response& resp) -> bool {
  if (cur_state_ != StateValue::Record) {
    resp.message = "Not in RECORD status, ignore record stop command.";
    return true;
  }

  cur_state_ = StateValue::Idle;
  if (!req.keep) {
    resp.message = "Do not keep teach path, discard recorded data.";
  } else {
    if (!write_file(file_path_)) {
      resp.message = "Write file failed.";
    } else {
      resp.message = "Keep teach path, save successful.";
    }
  }

  record_path_.clear();
  return true;
}

void StateMachine::record_path() {
  auto pose = robot_pose();
  if (!pose) return;

  if (record_path_.empty()) {
    record_path_.emplace_back(pose.value());
    return;
  }

  auto len = std::hypot(record_path_.back().position.x - pose->position.x,
                        record_path_.back().position.y - pose->position.y);
  auto agu = std::abs(tf2::getYaw(record_path_.back().orientation) - tf2::getYaw(pose->orientation));
  if (len < RECORD_PATH_LEN_DENS && agu < RECORD_PATH_AGU_DENS) return;
  record_path_.emplace_back(pose.value());
}

void StateMachine::visualize_poses(std::vector<geometry_msgs::Pose> const& poses, ros::Publisher const& pub) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  for (auto const& p : poses) {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose = p;
    path.poses.emplace_back(ps);
  }

  pub.publish(path);
}

auto StateMachine::check_file_exist(std::string& file_path) -> bool {
  std::ifstream exist(file_path.c_str());
  return !!exist;
}

auto StateMachine::write_file(std::string& file_path) -> bool {
  std::ofstream out(file_path.c_str());
  if (!out.is_open()) {
    ROS_ERROR("Open file %s failed!", file_path.c_str());
    return false;
  }

  int lines = 0;
  for (auto const& p : record_path_) {
    ++lines;
    out << std::to_string(p.position.x) << " " << std::to_string(p.position.y) << " " << std::to_string(tf2::getYaw(p.orientation)) << "\n";
  }

  out.close();
  ROS_INFO("Write path success. (%d poses)", lines);
  return true;
}

auto StateMachine::read_file(std::string& file_path) -> bool {
  std::ifstream in(file_path.c_str());
  if (!in.is_open()) {
    ROS_ERROR("Open file %s failed!", file_path.c_str());
    return false;
  }

  cur_route_.header.frame_id = "map";
  cur_route_.header.stamp = ros::Time::now();
  cur_route_.poses.clear();
  int lines = 0;
  std::string contend, temp;
  std::vector<std::string> temps;
  while (getline(in, contend)) {
    ++lines;
    temps.clear();
    temp.clear();

    for (auto const& c : contend) {
      if (c != ' ') {
        temp += c;
      } else {
        temps.emplace_back(temp);
        temp.clear();
      }
    }

    if (!temp.empty()) temps.emplace_back(temp);
    if (temps.size() != 3) {
      ROS_ERROR("%d line in %s file not correct!", lines, file_path.c_str());
      continue;
    }

    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = std::stod(temps[0]);
    p.pose.position.y = std::stod(temps[1]);
    p.pose.orientation.z = std::sin(std::stod(temps[2]) / 2.0);
    p.pose.orientation.w = std::cos(std::stod(temps[2]) / 2.0);
    cur_route_.poses.emplace_back(p);
  }

  ROS_INFO("Read path success. (%d poses)", lines);
  return !cur_route_.poses.empty();
}
}
