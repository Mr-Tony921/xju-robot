//
// Created by tony on 2022/12/2.
//

#pragma once

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace xju::pnc {
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define WHITE   "\033[37m"      /* White */

typedef struct point {
double x;
double y;
bool operator==(point const& rhs) const {
  return std::abs(x - rhs.x) < 1e-6 && std::abs(y - rhs.y) < 1e-6;
}
} point_t;

typedef struct line {
point_t a;
point_t b;
std::vector<int> go_list {};
} line_t;

using lines_type = std::vector<line_t>;

typedef struct graph {
double cost = std::numeric_limits<double>::max();
std::vector<geometry_msgs::PoseStamped> edge{};
} graph_t; // 没有用这个结构体实现，因为边都是直线，因此简单替换成长度了，如果支持复杂通道，需要把路径作为边存下来

constexpr static const int EXTRA_POINTS_NUM = 3;
constexpr static const double EXTRA_POINTS_RANGE = 5.0;

class HalfStructPlanner {
public:
  HalfStructPlanner();

  ~HalfStructPlanner();

  void init();

  void set_costmap(std::shared_ptr<costmap_2d::Costmap2D> const& costmap) {
    costmap_ = costmap;
  }

  void set_traffic_route(lines_type const& lines);

//  auto get_path(geometry_msgs::PoseStamped const& start, geometry_msgs::PoseStamped const& goal) -> nav_msgs::Path;
  auto get_path(geometry_msgs::PoseStamped const& start,
                geometry_msgs::PoseStamped const& goal) -> std::vector<geometry_msgs::PoseStamped>; // 将返回的点序用goto连接，不直接返回路径，不在这个类做地图路径搜索

  auto is_traffic_plan() -> bool;

private:
  void show_traffic_route();

  void show_graph();

  void calculate_pre_graph();

  void calculate_graph();

  auto nearest_point_of_segment(point_t const& p, point_t const& a, point_t const& b) -> point_t;

  auto distance(point_t const& a, point_t const& b) -> double;

  auto distance(geometry_msgs::PoseStamped const& a, geometry_msgs::PoseStamped const& b) -> double {
    auto pa = point_t {a.pose.position.x, a.pose.position.y};
    auto pb = point_t {b.pose.position.x, b.pose.position.y};
    return distance(pa, pb);
  }

  void print_graph();

  auto line_safe(point_t const& a, point_t const& b) -> bool;

  auto line_safe(geometry_msgs::PoseStamped const& a, geometry_msgs::PoseStamped const& b) -> bool {
    auto pa = point_t {a.pose.position.x, a.pose.position.y};
    auto pb = point_t {b.pose.position.x, b.pose.position.y};
    return line_safe(pa, pb);
  }

  auto dijkstra() -> std::vector<int>;

private:
  lines_type traffic_routes_;

  ros::Publisher vis_pub_;
  ros::Publisher res_pub_;

//  graph_type **graph_;
  double** graph_;
  double** graph_bk_;
  std::vector<geometry_msgs::PoseStamped> nodes_;

  std::shared_ptr<costmap_2d::Costmap2D> costmap_;

  int node_num_;
};
}
