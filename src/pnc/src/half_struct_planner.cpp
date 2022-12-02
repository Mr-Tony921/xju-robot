//
// Created by tony on 2022/12/2.
//

#include "half_struct_planner.h"

namespace xju::pnc {
void HalfStructPlanner::init() {
  ros::NodeHandle nh("/");
  vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("traffic_route", 1, true);
}

void HalfStructPlanner::set_traffic_route(lines_type const& lines) {
  traffic_routes_ = lines;
  show_traffic_route();
}

auto HalfStructPlanner::get_path(geometry_msgs::PoseStamped const& start, geometry_msgs::PoseStamped const& goal) -> nav_msgs::Path {
  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  path.poses.emplace_back(start);
  return path;
}

auto HalfStructPlanner::is_traffic_plan() -> bool {
  return !traffic_routes_.empty();
}

void HalfStructPlanner::show_traffic_route() {
  int cnt = 0;
  geometry_msgs::Point point;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.08;
  marker.scale.y = 0.2;
  marker.scale.z = 0.4;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.points.resize(2);
  for (auto const& line : traffic_routes_) {
    marker.id = cnt++;
    point.x = line.first.first;
    point.y = line.first.second;
    marker.points[0] = point;
    point.x = line.second.first;
    point.y = line.second.second;
    marker.points[1] = point;
    marker_array.markers.emplace_back(marker);
  }
  vis_pub_.publish(marker_array);
}

void HalfStructPlanner::calculate_pre_graph() {
  return;
}

void HalfStructPlanner::calculate_graph() {
  return;
}
}
