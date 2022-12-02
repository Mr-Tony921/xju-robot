//
// Created by tony on 2022/12/2.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace xju::pnc {
using lines_type = std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>>;

class HalfStructPlanner {
public:
  HalfStructPlanner() = default;

  ~HalfStructPlanner() = default;

  void init();

  void set_traffic_route(lines_type const& lines);

  auto get_path(geometry_msgs::PoseStamped const& start, geometry_msgs::PoseStamped const& goal) -> nav_msgs::Path;

  auto is_traffic_plan() -> bool;

private:
  void show_traffic_route();

  void calculate_pre_graph();

  void calculate_graph();

private:
  lines_type traffic_routes_;

  ros::Publisher vis_pub_;
};
}
