#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "Bezier.h"

namespace xju::planning {
class BezierTrajectoryGeneratorWaypoint {
public:
  explicit BezierTrajectoryGeneratorWaypoint(double step_in_fine_path) : step_in_fine_path_(step_in_fine_path) {}

  ~BezierTrajectoryGeneratorWaypoint() = default;

  auto calculate_optimized_path(nav_msgs::Path const& path) const -> nav_msgs::Path;

  /*将控制点转换为Bezier类需要的数据格式*/
  auto calculate_control_points(nav_msgs::Path const& path) const -> std::vector<Bezier::Point>;

  /*根据位置关系计算nav_msgs::Path的方向数据*/
  static auto calculate_path_orientation(nav_msgs::Path const& path,
                                         bool loop_back = false) -> std::optional<nav_msgs::Path>;

  static inline void calculate_direction(geometry_msgs::PoseStamped const& pose1,
                                         geometry_msgs::PoseStamped& pose2) {
    auto theta = atan2(pose1.pose.position.y - pose2.pose.position.y,
                       pose1.pose.position.x - pose2.pose.position.x);
    pose2.pose.orientation.w = cos(theta / 2);
    pose2.pose.orientation.z = sin(theta / 2);
  }

private:
  double step_in_fine_path_;
};
}
