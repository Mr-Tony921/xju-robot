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
    recording_(false) {
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
  ROS_INFO("Xju state machine initialized!");
}

void StateMachine::run() {
  ros::Rate r(ros::Duration(0.1));
  while(ros::ok()) {
    ros::spinOnce();
    if (recording_) {
      record_path();
      visualize_poses(record_path_, record_path_pub_);
    }

    r.sleep();
  }
}

void StateMachine::stop() {
  pub_zero_vel();
}

void StateMachine::pub_zero_vel() {
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.angular.z = 0;
  vel_pub_.publish(vel);
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

auto
StateMachine::record_start_service(xju_pnc::record_start::Request& req, xju_pnc::record_start::Response& resp) -> bool {
  if (recording_) {
    resp.message = "Already in recording process, ignore this operation.";
    return true; // return false will miss message in response
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
    dir = "/home/tony/course_ws/path";
  }

  file_path_ = dir + "/" + name;
  if (check_file_exist(file_path_)) {
    resp.message = "File already exist, ignore this operation.";
    return true;
  }

  record_path_.clear();
  recording_ = true;
  resp.message = "Start recording path, save to " + file_path_ + ".";
  return true;
}

auto
StateMachine::record_stop_service(xju_pnc::record_stop::Request& req, xju_pnc::record_stop::Response& resp) -> bool {
  if (!recording_) {
    resp.message = "Not in recording process, ignore this operation.";
    return true;
  }

  recording_ = false;
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
}
