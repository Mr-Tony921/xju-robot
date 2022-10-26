//
// Created by tony on 2022/10/25.
//

#include "PathPlanningNode.h"

namespace xju::planning {
PathPlanning::PathPlanning() :
  grid_ready_(false),
  grid_updated_ready_(false) {
  ros::NodeHandle nh("/");
  costmap_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("move_base_flex/global_costmap/costmap", 1,
                                                       &PathPlanning::grid_callback, this);
  costmap_updated_sub_ = nh.subscribe<map_msgs::OccupancyGridUpdate>(
    "move_base_flex/global_costmap/costmap_updates", 1,
    &PathPlanning::grid_updated_callback, this);
  get_path_in_zone_srv_ = nh.advertiseService("xju_zone_path", &PathPlanning::get_path_in_zone, this);
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("debug_path", debug_path_, std::string(""));
#ifdef DEBUG_MODE
  debug_ = nh.advertise<nav_msgs::Path>("/debug_path", 1, true);
#endif
}

void PathPlanning::grid_callback(nav_msgs::OccupancyGridConstPtr const& msg) {
  grid_ready_ = false;
  grid_ = *msg;
  grid_ready_ = true;
}

void PathPlanning::grid_updated_callback(map_msgs::OccupancyGridUpdateConstPtr const& msg) {
  if (!grid_ready_) return;

  grid_updated_ready_ = false;
  int index = 0;
  for (unsigned y = msg->y; y < msg->y + msg->height; ++y) {
    for (unsigned x = msg->x; x < msg->x + msg->width; ++x) {
      grid_.data[y * grid_.info.width + x] = msg->data[index++];
    }
  }

  grid_updated_ready_ = true;
}

auto PathPlanning::get_path_in_zone(coverage_path_planner::GetPathInZone::Request& req,
                                    coverage_path_planner::GetPathInZone::Response& resp) -> bool {
  ROS_INFO("getPathInZone: receive call request.");
  req.zone = BezierTrajectoryGeneratorWaypoint::calculate_path_orientation(req.zone, true).value();

  for (auto& p: req.zone.poses) {
    ROS_INFO("[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
             p.pose.position.x,
             p.pose.position.y,
             p.pose.orientation.x,
             p.pose.orientation.y,
             p.pose.orientation.z,
             p.pose.orientation.w);
  }

//  grid_updated_ready_ = false;
  auto frequency = 4.0;
  ros::Rate wait_rate(frequency);
  auto try_count = 0;
  while (!grid_updated_ready_) {
    ROS_WARN("Global costmap not updated!");
    if ((++try_count) / frequency > 30) return false;
    ros::spinOnce();
    wait_rate.sleep();
  }

  auto coverage_ptr = std::make_unique<CoveragePathPlanner>(grid_, req.zone, req.type);
  resp.coverage_paths = coverage_ptr->get_path_interface();
#ifdef DEBUG_MODE
  resp.coverage_paths[0].path.header.stamp    = ros::Time().now();
  resp.coverage_paths[0].path.header.frame_id = "map";
  auto pub_path = resp.coverage_paths[0].path;
  for (auto i = 1; i < resp.coverage_paths.size(); ++i) {
      pub_path.poses.insert(pub_path.poses.end(),
                            resp.coverage_paths[i].path.poses.begin(),
                            resp.coverage_paths[i].path.poses.end());
  }

  debug_.publish(pub_path);
#endif
  return !resp.coverage_paths.empty();
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, xju::planning::PLANNER_NODE_NAME);
  xju::planning::PathPlanning path_planning;

  ros::spin();
  ros::waitForShutdown();
  return 0;
}
