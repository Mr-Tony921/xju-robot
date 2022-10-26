//
// Created by tony on 2022/10/25.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <iostream>
#include <std_srvs/Empty.h>

#include "CoveragePathPlanner.h"
#include "coverage_path_planner/GetPathInZone.h"

namespace xju::planning {
#define RECORD_IN_FILE 0
//#define DEBUG_MODE

static const std::string PLANNER_NODE_NAME = "path_planning_node";

class PathPlanning {
public:
  PathPlanning();

  ~PathPlanning() = default;

private:
  void grid_callback(const nav_msgs::OccupancyGridConstPtr& msg);

  void grid_updated_callback(const map_msgs::OccupancyGridUpdateConstPtr& msg);

  auto get_path_in_zone(coverage_path_planner::GetPathInZone::Request& req,
                        coverage_path_planner::GetPathInZone::Response& resp) -> bool;

private:

  ros::ServiceServer get_path_in_zone_srv_;

  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_updated_sub_;
#ifdef DEBUG_MODE
  ros::Publisher debug_;
#endif

  nav_msgs::OccupancyGrid grid_;

  std::atomic_bool grid_ready_;
  std::atomic_bool grid_updated_ready_;

  std::string debug_path_;
};
}
