//
// Created by tony on 2022/10/11.
//

#include <chrono>
#include <fstream>
#include "state_machine.h"

namespace xju::pnc {

uint8_t* StateMachine::cost_translation_ = nullptr;

StateMachine::StateMachine()
  : tf_(new tf2_ros::Buffer()),
    tfl_(new tf2_ros::TransformListener(*tf_)),
    goto_ctrl_(new GotoCtrl("move_base_flex/move_base")),
    exe_ctrl_(new ExeCtrl("move_base_flex/exe_path")),
    cur_state_(StateValue::Idle),
    running_state_(RunStateValue::Goto),
    cur_index_(0),
    obs_index_(0) {
  if (!cost_translation_) {
    cost_translation_ = new uint8_t[101];
    for (int i = 0; i < 101; ++i) {
      cost_translation_[i] = static_cast<uint8_t>(i * 254 / 100);
    }
  }

  half_struct_planner_ = std::make_shared<HalfStructPlanner>();
  half_struct_planner_->init();
}

StateMachine::~StateMachine() {
  stop();
}

void StateMachine::init() {
  ros::NodeHandle nh;
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  record_path_pub_ = nh.advertise<nav_msgs::Path>("record_path", 1);
  record_point_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("record_zone", 1);
  normal_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  cur_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
  point_sub_ = nh.subscribe(RVIZ_POINT, 1, &StateMachine::point_cb, this);
  costmap_sub_ = nh.subscribe(COSTMAP, 5, &StateMachine::costmap_cb, this);
  costmap_update_sub_ = nh.subscribe(COSTMAP_UPDATE, 5, &StateMachine::costmap_update_cb, this);
  traffic_goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/traffic_goal", 1, &StateMachine::traffic_goal_cb, this);
  task_srv_ = nh.advertiseService(TASK_SRV, &StateMachine::task_service, this);
  coverage_srv_ = nh.serviceClient<coverage_path_planner::GetPathInZone>("/xju_zone_path");
  ROS_ERROR_COND(!goto_ctrl_->waitForServer(ros::Duration(5.0)), "move base action not online!");
  ROS_ERROR_COND(!exe_ctrl_->waitForServer(ros::Duration(5.0)), "exe path action not online!");
  record_path_.header.frame_id = "map";
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
        record_path_pub_.publish(record_path_);
        break;
      case StateValue::Run:
        running_state();
        break;
      default:
        ROS_ERROR("Error StateValue!");
        break;
    }

    r.sleep();
  }
}

void StateMachine::running_state() {
  using goalState = actionlib::SimpleClientGoalState;
  static int wait_cnt = 0;
  switch (running_state_) {
    // 1 跟线状态
    case RunStateValue::Follow: {
      // 1.1 跟线是否成功？是-任务结束（成功）
      if (exe_ctrl_->getState() == goalState::SUCCEEDED
          /*&& cur_route_.poses.size() - cur_index_ < 10
          && distance(robot_pose().value(), cur_route_.poses.back().pose).first < 0.5*/) {
        if (routes_.empty()) {
          ROS_INFO("Task success!");
          reset();
        } else {
          // 1.1.2 切换路径
          cur_route_ = routes_.front();
          cur_index_ = 0;
          routes_.pop_front();
          running_state_ = RunStateValue::Goto;
          send_goto(cur_index_);
        }
        return;
      }
      // 1.2 跟线是否失败？是-切换至点到点状态
      if (exe_ctrl_->getState().isDone()) {
        for (auto i = cur_index_; i < cur_route_.poses.size(); ++i) {
          if (is_free(cur_route_.poses.at(i))) {
            cur_index_ = i;
            break;
          }
        }

        cur_index_ = std::min(cur_index_, cur_route_.poses.size() - 1);
        cur_pose_pub_.publish(cur_route_.poses.at(cur_index_));
        ROS_INFO("Exe path failed, start goto, target %lu", cur_index_);
        running_state_ = RunStateValue::Goto;
        send_goto(cur_index_);
      }
      // 1.3 更新机器人路径索引
      cur_index_ = nearest_info(cur_route_, cur_index_, 0, 20, true).first;
      cur_pose_pub_.publish(cur_route_.poses.at(cur_index_));
      // 1.4 前方路点是否有障碍？是-切换至避障等待状态
      for (auto i = 0; i < PATH_SAFE_DIS_NUM; ++i) {
        if (cur_index_ + i >= cur_route_.poses.size()) break;
        if (is_danger(cur_route_.poses.at(cur_index_ + i))) {
          obs_index_ = cur_index_ + i;
          ROS_WARN("Obs found at %.3f away",
                   distance(cur_route_.poses[cur_index_].pose, cur_route_.poses[obs_index_].pose).first);
          stop();
          wait_cnt = 0;
          running_state_ = RunStateValue::Wait;
        }
      }
      break;
    }
    // 2 避障等待状态
    case RunStateValue::Wait: {
      // 2.1 前方路点是否仍旧为障碍？否-切换至跟线状态
      auto still_danger = false;
      for (auto i = 0; i < 10; ++i) {
        if (obs_index_ + i >= cur_route_.poses.size()) break;
        if (is_danger(cur_route_.poses.at(obs_index_ + i))) {
          still_danger = true;
          break;
        }
      }

      if (!still_danger) {
        ROS_INFO("Obs clear, keep moving");
        running_state_ = RunStateValue::Follow;
        send_exe(cur_index_);
        break;
      }
      // 2.2 等待是否超时？是-切换至点到点状态
      if (wait_cnt++ > WAIT_COUNT) {
        for (auto i = obs_index_; i < cur_route_.poses.size(); ++i) {
          if (is_free(cur_route_.poses.at(i))) {
            cur_index_ = i;
            break;
          }
        }

        cur_index_ = std::min(cur_index_, cur_route_.poses.size() - 1);
        cur_pose_pub_.publish(cur_route_.poses.at(cur_index_));
        ROS_INFO("Wait finish, start avoid, target %lu", cur_index_);
        running_state_ = RunStateValue::Goto;
        send_goto(cur_index_);
      }
      break;
    }
    // 3 点到点状态
    case RunStateValue::Goto: {
      // 3.1 点到点是否完成？是-切换至跟线状态
      if (goto_ctrl_->getState() == goalState::SUCCEEDED
          /*&& distance(robot_pose().value(), cur_route_.poses[cur_index_].pose).first < 0.1
          && distance(robot_pose().value(), cur_route_.poses[cur_index_].pose).second < 10 * DEG2RAD*/) {
        running_state_ = RunStateValue::Follow;
        send_exe(cur_index_);
        break;
      }
      // 3.2 点到点是否失败及目标点是否安全？否-前向寻找安全点
      if (goto_ctrl_->getState().isDone()
          || !is_free(cur_route_.poses.at(cur_index_))) {
        for (auto i = cur_index_ + 10; i < cur_route_.poses.size(); ++i) {
          if (is_free(cur_route_.poses.at(i))) {
            cur_index_ = i;
            break;
          }
        }

        // 3.3 路径上是否仍有可用点？是-更新目标点；否-任务结束（失败）
        if (cur_index_ >= cur_route_.poses.size()) {
          if (routes_.empty()) {
            ROS_WARN("Task failed!");
            reset();
          } else {
            // 3.3.2 切换路径
            cur_route_ = routes_.front();
            cur_index_ = 0;
            routes_.pop_front();
            send_goto(cur_index_);
          }
          return;
        }

        cur_pose_pub_.publish(cur_route_.poses.at(cur_index_));
        ROS_WARN("Target not safe, update %lu", cur_index_);
        send_goto(cur_index_);
      }
      break;
    }
    // 错误状态（不会进来）
    default:
      ROS_ERROR("Error RunStateValue!");
      break;
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
  running_state_ = RunStateValue::Goto;
  cur_state_ = StateValue::Idle;
  cur_index_ = 0;
  cur_route_.poses.clear();
  routes_.clear();
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

auto StateMachine::nearest_info(nav_msgs::Path const& path, size_t const& index, int lb, int rb, bool following)
-> std::pair<size_t, double> {
  size_t idx;
  auto robot_pos = robot_pose();
  if (!robot_pos) return std::make_pair(index, 0.0);

  auto dis = std::numeric_limits<double>::max();
  lb = std::max(static_cast<int>(index) - lb, 0);
  rb = std::min(index + rb, path.poses.size());
  if (following &&  distance(robot_pos.value(), path.poses.at(index).pose).first > 1.0) {
    rb = path.poses.size(); // maybe fatal when global path too long!
  }

  for (auto i = lb; i < rb; ++i) {
    auto err = distance(robot_pos.value(), path.poses[i].pose);
    if (following) err.second = 0;
    if (err.first < dis && err.second < 30 * DEG2RAD) {
      dis = err.first;
      idx = static_cast<size_t>(i);
    }
  }

  return std::make_pair(idx, dis);
}

void StateMachine::point_cb(geometry_msgs::PointStampedConstPtr const& msg) {
  if (cur_state_ != StateValue::Record) return;

  record_points_.emplace_back(*msg);
  geometry_msgs::PolygonStamped polygon;
  polygon.header.frame_id = "map";
  polygon.header.stamp = ros::Time::now();
  geometry_msgs::Point32 point32;
  for (auto const& p : record_points_) {
    point32.x = p.point.x;
    point32.y = p.point.y;
    point32.z = 0;
    polygon.polygon.points.emplace_back(point32);
  }
  record_point_pub_.publish(polygon);
}

void StateMachine::costmap_cb(nav_msgs::OccupancyGrid::ConstPtr const& msg) {
  std::lock_guard<std::mutex> lock(map_update_mutex_);
  if (!costmap_) {
    ROS_WARN("Initiate new costmap");
    costmap_ = std::make_shared<costmap_2d::Costmap2D>(msg->info.width, msg->info.height, msg->info.resolution,
                                                       msg->info.origin.position.x, msg->info.origin.position.y);
  } else {
    ROS_WARN("Update costmap!");
    costmap_->resizeMap(msg->info.width, msg->info.height, msg->info.resolution,
                        msg->info.origin.position.x, msg->info.origin.position.y);
  }

  uint32_t x;
  for (uint32_t y = 0; y < msg->info.height; y++) {
    for (x = 0; x < msg->info.width; x++) {
      if (msg->data[y * msg->info.width + x] < 0) {
        costmap_->setCost(x, y, costmap_2d::NO_INFORMATION);
        continue;
      }
      costmap_->setCost(x, y, cost_translation_[msg->data[y * msg->info.width + x]]);
    }
  }
}

void StateMachine::costmap_update_cb(map_msgs::OccupancyGridUpdate::ConstPtr const& msg) {
  std::lock_guard<std::mutex> lock(map_update_mutex_);
  if (!costmap_) {
    ROS_WARN("Costmap not initiate yet");
    return;
  }

  if (msg->width * msg->height != msg->data.size()
      || msg->x + msg->width > costmap_->getSizeInCellsX()
      || msg->y + msg->height > costmap_->getSizeInCellsY()) {
    ROS_ERROR("Costmap update got invalid data set");
    return;
  }

  size_t index = 0;
  int x;
  for (auto y = msg->y; y < msg->y + msg->height; y++) {
    for (x = msg->x; x < msg->x + msg->width; x++) {
      if (msg->data[index] < 0) {
        costmap_->setCost(x, y, costmap_2d::NO_INFORMATION);
        index++;
        continue;
      }
      costmap_->setCost(x, y, cost_translation_[msg->data[index++]]);
    }
  }
}

void StateMachine::traffic_goal_cb(geometry_msgs::PoseStamped::ConstPtr const& msg) {
  if (!half_struct_planner_ || !half_struct_planner_->is_traffic_plan()) {
    normal_goal_pub_.publish(*msg);
    return;
  }

  half_struct_planner_->set_costmap(costmap_);
  routes_.clear();
  geometry_msgs::PoseStamped start, goal;
  start.header.frame_id = "map";
  start.header.stamp = ros::Time::now();
  start.pose = robot_pose().value();
  goal = *msg;
  auto poses = half_struct_planner_->get_path(start, goal);
  if (poses.empty()) {
    ROS_ERROR("Traffic path not found!");
    return;
  }

  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.poses.resize(1);
  for (auto const& pose : poses) {
    path.poses.front() = pose;
    routes_.emplace_back(path);
  }

  cur_route_ = routes_.front();
  routes_.erase(routes_.begin());
  cur_index_ = 0;
  cur_state_ = StateValue::Run;
  running_state_ = RunStateValue::Goto;
  send_goto(cur_index_);
}

auto StateMachine::is_free(const geometry_msgs::PoseStamped& pose) const -> bool {
  auto cost = pose_cost(pose);
  return cost < 66;
}

auto StateMachine::is_danger(const geometry_msgs::PoseStamped& pose) const -> bool {
  auto cost = pose_cost(pose);
  return cost >= 253;
}

auto StateMachine::pose_cost(const geometry_msgs::PoseStamped& pose) const -> uint8_t {
  if (!costmap_) {
    ROS_WARN("No costmap yet");
    return true;
  }

  uint32_t mx, my;
  if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    ROS_WARN("Can't find point mx, my in cost map");
    return 0;
  }

  return costmap_->getCost(mx, my);
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
  nav_msgs::Path route;
  route.header = cur_route_.header;
  route.poses.assign(cur_route_.poses.begin() + index, cur_route_.poses.end());
  goal.path = route;
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

auto StateMachine::task_service(xju_pnc::xju_task::Request& req, xju_pnc::xju_task::Response& resp) -> bool {
  std::string root_path;
  ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
  switch (req.type) {
    case xju_pnc::xju_task::Request::EXECUTE: {
      if (cur_state_ == StateValue::Record) {
        resp.message = "Recording path now, ignore task_service.";
        return true; // return false will miss message in response
      }

      if (req.command == xju_pnc::xju_task::Request::PAUSE) {
        if (cur_state_ != StateValue::Run) {
          resp.message = "Not in RUNNING status, ignore pause command.";
        } else {
          cur_state_ = StateValue::Pause;
          stop();
          resp.message = "Task pause.";
        }
        return true;
      }

      if (req.command == xju_pnc::xju_task::Request::STOP) {
        if (cur_state_ != StateValue::Run && cur_state_ != StateValue::Pause) {
          resp.message = "Not in RUNNING and PAUSE status, ignore stop command.";
        } else {
          reset();
          resp.message = "Task stop.";
        }
        return true;
      }

      if (req.command == xju_pnc::xju_task::Request::START) {
        if (cur_state_ == StateValue::Pause) {
          resp.message = "Task resume.";
          auto nearest = nearest_info(cur_route_, cur_index_, 0, 20);
          cur_index_ = nearest.second < 0.1 ? nearest.first : cur_index_;
          cur_state_ = StateValue::Run;
          running_state_ = RunStateValue::Goto;
          send_goto(cur_index_);
          return true;
        }

        reset();
        auto dir = req.dir;
        if (dir == "") dir = root_path + "/path/";
        file_path_ = dir + req.path_name;
        if (!check_file_exist(file_path_)) {
          resp.message = file_path_ + " not exist, ignore start command.";
          return true;
        }

        if (!read_file(file_path_)) {
          resp.message = "Read path in file failed.";
          return true;
        }

        cur_route_ = routes_.front();
        routes_.pop_front();
        cur_state_ = StateValue::Run;
        running_state_ = RunStateValue::Goto;
        send_goto(cur_index_);
        resp.message = "Start new task!";
        return true;
      }

      resp.message = "Wrong exe service command, do nothing.";
      return true;
    }
    case xju_pnc::xju_task::Request::RECORD: {
      switch (req.command) {
        case xju_pnc::xju_task::Request::START: {
          if (cur_state_ != StateValue::Idle) {
            resp.message = "Not in IDLE status, ignore record command.";
            return true;
          }

          auto name = req.path_name;
          if (name == "") {
            auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::stringstream ss;
            ss << "path_" << std::put_time(std::localtime(&t), "%F %T");
            name = ss.str();
          }

          auto dir = req.dir;
          if (dir == "") {
            dir = root_path + "/path/";
          }

          file_path_ = dir + name;
          if (check_file_exist(file_path_)) {
            resp.message = "File already exist, ignore this operation.";
            return true;
          }

          record_path_.poses.clear();
          record_points_.clear();
          cur_state_ = StateValue::Record;
          resp.message = "Start recording path, save to " + file_path_ + ".";
          return true;
        }
        case xju_pnc::xju_task::Request::KEEP_TEACH:
        case xju_pnc::xju_task::Request::KEEP_COVER_ZZ:
        case xju_pnc::xju_task::Request::KEEP_COVER_BS:
        case xju_pnc::xju_task::Request::DISCARD: {
          if (cur_state_ != StateValue::Record) {
            resp.message = "Not in RECORD status, ignore record stop command.";
            return true;
          }

          cur_state_ = StateValue::Idle;
          if (req.command == xju_pnc::xju_task::Request::DISCARD) {
            resp.message = "Do not keep path, discard recorded data.";
          } else {
            if (req.command == xju_pnc::xju_task::Request::KEEP_TEACH) {
              std::vector<nav_msgs::Path> paths{record_path_};
              if (!write_file(file_path_, paths)) {
                resp.message = "Write file failed.";
              } else {
                resp.message = "Keep teach path, save successful.";
              }
            } else {
              if (!record_points_.empty()) {
                ROS_INFO("Got %lu record points, discard record path!", record_points_.size());
                record_path_.poses.clear();
                geometry_msgs::PoseStamped pose;
                pose.pose.orientation.w = 1.0;
                for (auto const& p : record_points_) {
                  pose.pose.position = p.point;
                  record_path_.poses.emplace_back(pose);
                }
              } else {
                ROS_INFO("No record point, use record path %lu", record_path_.poses.size());
              }
              coverage_path_planner::GetPathInZone r{};
              r.request.zone = record_path_;
              r.request.type = req.command == xju_pnc::xju_task::Request::KEEP_COVER_ZZ ? 0 : 1;
              if (coverage_srv_.call(r)) {
                ROS_INFO("Coverage service call success, got %lu paths", r.response.coverage_paths.size());
                record_path_.poses.clear();
                for (auto const& path : r.response.coverage_paths) {
                  for (auto const& p : path.poses) {
                    record_path_.poses.emplace_back(p);
                  }
                }
                record_path_pub_.publish(record_path_);
              }

              if (!write_file(file_path_, r.response.coverage_paths)) {
                resp.message = "Write file failed.";
              } else {
                resp.message = "Keep cover path, save successful.";
              }
            }
          }

          record_path_.poses.clear();
          record_points_.clear();
          return true;
        }
        case xju_pnc::xju_task::Request::KEEP_TRAFFIC_ROUTE : {
          auto dir = root_path + "/map/";
          auto map_path = dir + req.map + "_traffic_route.txt";
          if (!write_traffic_route(map_path, record_points_)) {
            resp.message = "Write fie failed.";
          } else {
            resp.message = "Keep traffic route successful.";
          }

          record_path_.poses.clear();
          record_points_.clear();
          return true;
        }
        default:
          ROS_ERROR("Illegal service command in record %d", req.command);
          break;
      }
      return true;
    }
    case xju_pnc::xju_task::Request::LOAD_TRAFFIC_ROUTE: {
      lines_type lines;
      auto dir = root_path + "/map/";
      auto map_path = dir + req.map + "_traffic_route.txt";
      if (!read_traffic_route(map_path, lines)) {
        resp.message = "read file failed.";
      } else {
        resp.message = "read file successful, got " + std::to_string(lines.size()) + " lines";
      }

      half_struct_planner_->set_traffic_route(lines);
      return true;
    }
    default: {
      ROS_ERROR("Illegal service type %d", req.type);
      break;
    }
  }

  return true;
}

void StateMachine::record_path() {
  auto pose = robot_pose();
  if (!pose) return;

  geometry_msgs::PoseStamped ps;
  if (record_path_.poses.empty()) {
    ps.pose = pose.value();
    record_path_.poses.emplace_back(ps);
    return;
  }

  auto len = std::hypot(record_path_.poses.back().pose.position.x - pose->position.x,
                        record_path_.poses.back().pose.position.y - pose->position.y);
  auto agu = std::abs(tf2::getYaw(record_path_.poses.back().pose.orientation) - tf2::getYaw(pose->orientation));
  if (len < RECORD_PATH_LEN_DENS && agu < RECORD_PATH_AGU_DENS) return;
  ps.pose = pose.value();
  record_path_.poses.emplace_back(ps);
}

auto StateMachine::check_file_exist(std::string& file_path) -> bool {
  std::ifstream exist(file_path.c_str());
  return !!exist;
}

auto StateMachine::write_file(std::string& file_path, std::vector<nav_msgs::Path> const& paths) -> bool {
  std::ofstream out(file_path.c_str());
  if (!out.is_open()) {
    ROS_ERROR("Open file %s failed!", file_path.c_str());
    return false;
  }

  for (auto const& path : paths) {
    for (auto const& p : path.poses) {
      out << std::to_string(p.pose.position.x) << " "
          << std::to_string(p.pose.position.y) << " "
          << std::to_string(tf2::getYaw(p.pose.orientation)) << "\n";
    }
    out << "EOP" << "\n";
  }

  out.close();
  return true;
}

auto StateMachine::read_file(std::string& file_path) -> bool {
  std::ifstream in(file_path.c_str());
  if (!in.is_open()) {
    ROS_ERROR("Open file %s failed!", file_path.c_str());
    return false;
  }

  nav_msgs::Path route;
  route.header.frame_id = "map";
  route.header.stamp = ros::Time::now();
  std::string contend, temp;
  std::vector<std::string> temps;
  record_path_.poses.clear();
  while (getline(in, contend)) {
    if (contend == "EOP" && !route.poses.empty()) {
      ROS_INFO("Route %lu got %lu poses", routes_.size() + 1, route.poses.size());
      routes_.emplace_back(route);
      route.poses.clear();
      continue;
    }

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
    if (temps.size() != 3) continue;
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = std::stod(temps[0]);
    p.pose.position.y = std::stod(temps[1]);
    p.pose.orientation.z = std::sin(std::stod(temps[2]) / 2.0);
    p.pose.orientation.w = std::cos(std::stod(temps[2]) / 2.0);
    route.poses.emplace_back(p);
    record_path_.poses.emplace_back(p);
  }

  record_path_pub_.publish(record_path_); // for debug
  record_path_.poses.clear();
  return !routes_.empty();
}

auto StateMachine::write_traffic_route(std::string& file_path, std::vector<geometry_msgs::PointStamped> const& line) -> bool {
  if (line.size() % 2 != 0) {
    ROS_ERROR("%lu size of line points is record, currently not support.", line.size());
    return false;
  }

  std::ofstream out(file_path.c_str(), std::ios_base::out | std::ios_base::app);
  if (!out.is_open()) {
    ROS_ERROR("Open file %s failed!", file_path.c_str());
    return false;
  }

  for (auto i = 0; i < line.size(); i += 2) {
    out << std::to_string(line[i].point.x) << " " << std::to_string(line[i].point.y)
        << " === "
        << std::to_string(line[i + 1].point.x) << " " << std::to_string(line[i + 1].point.y)
        << "\n";
  }

  out.close();
  return true;
}

auto StateMachine::read_traffic_route(std::string& file_path, lines_type& lines) -> bool {
  std::ifstream in(file_path.c_str());
  if (!in.is_open()) {
    ROS_ERROR("Open file %s failed!", file_path.c_str());
    return false;
  }

  line_t line;
  std::string contend, temp;
  std::vector<std::string> temps;
  while (getline(in, contend)) {
    temps.clear();
    temp.clear();
    for (auto const& c : contend) {
      if (c != ' ' && c != '=') {
        temp += c;
      } else if (!temp.empty()) {
        temps.emplace_back(temp);
        temp.clear();
      }
    }
    if (!temp.empty()) temps.emplace_back(temp);
    if (temps.size() < 4) continue;
    line.a.x = std::stod(temps[0]);
    line.a.y = std::stod(temps[1]);
    line.b.x = std::stod(temps[2]);
    line.b.y = std::stod(temps[3]);
    line.go_list.clear();
    for (auto i = 4; i < temps.size(); ++i) {
      line.go_list.emplace_back(std::stoi(temps[i]));
    }
    lines.emplace_back(line);
  }

  return !lines.empty();
}
}
