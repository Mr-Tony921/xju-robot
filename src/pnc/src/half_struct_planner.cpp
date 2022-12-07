//
// Created by tony on 2022/12/2.
//

#include "half_struct_planner.h"

namespace xju::pnc {
HalfStructPlanner::HalfStructPlanner() : node_num_(0),
                                         costmap_(nullptr),
                                         graph_(nullptr),
                                         graph_bk_(nullptr) {
  ROS_INFO("Start HalfStructPlanner!");
}

HalfStructPlanner::~HalfStructPlanner() {
  if (node_num_ == 0) return;

  for (auto i = 0; i < node_num_; ++i) {
    delete [] graph_[i];
    delete [] graph_bk_[i];
  }

  delete [] graph_;
  delete [] graph_bk_;
};

void HalfStructPlanner::init() {
  ros::NodeHandle nh("/");
  vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("traffic_route", 1);
  res_pub_ = nh.advertise<nav_msgs::Path>("traffic_route_points", 1);
}

void HalfStructPlanner::set_traffic_route(lines_type const& lines) {
  traffic_routes_ = lines;
  show_traffic_route();
  calculate_pre_graph();
}

auto HalfStructPlanner::get_path(geometry_msgs::PoseStamped const& start,
                                 geometry_msgs::PoseStamped const& goal) -> std::vector<geometry_msgs::PoseStamped> {
  std::vector<geometry_msgs::PoseStamped> result {};
  if (!is_traffic_plan()) return result;

  for (auto i = 0; i < node_num_; ++i) {
    for (auto j = 0; j < node_num_; ++j) {
      graph_[i][j] = graph_bk_[i][j];
    }
  }
  // 0 起点 1 终点
  nodes_[0] = start;
  nodes_[1] = goal;
  calculate_graph();
  auto node_list = dijkstra();
  for (auto const& node : node_list) {
    result.emplace_back(nodes_[node]);
  }
  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  path.poses = result;
  path.poses.emplace(path.poses.begin(), nodes_[0]);
  res_pub_.publish(path);

  return result;
}

auto HalfStructPlanner::is_traffic_plan() -> bool {
  return !traffic_routes_.empty();
}

void HalfStructPlanner::show_traffic_route() {
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  int serial = 0;
  geometry_msgs::Point p1, p2;
  geometry_msgs::Pose p;
  auto make_arrow_marker = [&](double r, double g, double b, geometry_msgs::Point const& p1, geometry_msgs::Point const& p2) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.08;
    marker.scale.y = 0.2;
    marker.scale.z = 0.4;
    marker.color.a = 0.5;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose.orientation.w = 1.0;
    marker.points.resize(2);
    marker.ns = "arrow";
    marker.id = id++;
    marker.points[0] = p1;
    marker.points[1] = p2;
    marker_array.markers.emplace_back(marker);
  };
  auto make_text_marker = [&](double r, double g, double b, geometry_msgs::Pose const& p) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.z = 1.0;
    marker.color.a = 0.5;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose = p;
    marker.ns = "text";
    marker.id = id++;
    marker.text = std::to_string(serial++);
    marker_array.markers.emplace_back(marker);
  };
  for (auto const& line : traffic_routes_) {
    p1.x = line.a.x;
    p1.y = line.a.y;
    p2.x = line.b.x;
    p2.y = line.b.y;
    p.position.x = (p1.x + p2.x) / 2;
    p.position.y = (p1.y + p2.y) / 2;
    auto o = std::atan2(p2.y - p1.y, p2.x - p2.x);
    p.orientation.z = std::sin(o / 2);
    p.orientation.w = std::cos(o / 2);
    make_arrow_marker(0.0, 1.0, 0.0, p1, p2);
    make_text_marker(0.0, 0.0, 0.0, p);
    if (line.go_list.empty()) return;
    for (auto const& node : line.go_list) {
      if (node >= traffic_routes_.size()) continue;
      p1.x = traffic_routes_[node].a.x;
      p1.y = traffic_routes_[node].a.y;
      make_arrow_marker(0.0, 0.0, 1.0, p2, p1);
    }
  }
  vis_pub_.publish(marker_array);
}

void HalfStructPlanner::show_graph() {
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  geometry_msgs::Point p1, p2;
  geometry_msgs::Pose p;
  p.orientation.w = 1.0;
  auto make_arrow_marker = [&](double r, double g, double b, geometry_msgs::Point const& p1, geometry_msgs::Point const& p2) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.08;
    marker.scale.y = 0.2;
    marker.scale.z = 0.4;
    marker.color.a = 0.5;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose.orientation.w = 1.0;
    marker.points.resize(2);
    marker.ns = "arrow";
    marker.id = id++;
    marker.points[0] = p1;
    marker.points[1] = p2;
    marker_array.markers.emplace_back(marker);
  };
  auto make_text_marker = [&](double r, double g, double b, geometry_msgs::Pose const& p, std::string const& t) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.z = 1.0;
    marker.color.a = 0.5;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose = p;
    marker.ns = "text";
    marker.id = id++;
    marker.text = t;
    marker_array.markers.emplace_back(marker);
  };
  for (auto i = 0; i < node_num_; ++i) {
    p.position.x = nodes_[i].pose.position.x;
    p.position.y = nodes_[i].pose.position.y;
    make_text_marker(0.0, 0.0, 0.0, p, std::to_string(i));
    for (auto j = 0; j < node_num_; ++j) {
      if (graph_[i][j] == std::numeric_limits<double>::max()) continue;
      p1.x = nodes_[i].pose.position.x;
      p1.y = nodes_[i].pose.position.y;
      p2.x = nodes_[j].pose.position.x;
      p2.y = nodes_[j].pose.position.y;
      make_arrow_marker(1.0, 0.0, 0.0, p1, p2);
    }
  }
  vis_pub_.publish(marker_array);
}

void HalfStructPlanner::calculate_pre_graph() {
  if (traffic_routes_.empty()) {
    ROS_WARN("Receive empty traffic routes, planner not working!");
    return;
  }

  // 快速通道的起终点数目+真实起终点+带来的新节点
  node_num_ = static_cast<int>(traffic_routes_.size() * 2 + 2 + EXTRA_POINTS_NUM * 2);
  // 0: 起点 1: 终点
  // 2 - 1+EXTRA_POINTS_NUM: 起点带来的新节点
  // 2+EXTRA_POINTS_NUM - 1+2*EXTRA_POINTS_NUM: 终点带来的新节点
  // 2+2*EXTRA_POINTS_NUM - end: traffic routes节点 (本函数处理)
  nodes_.resize(node_num_);
  graph_ = new double *[node_num_];
  graph_bk_ = new double *[node_num_];
  for (auto i = 0; i < node_num_; ++i) {
    graph_[i] = new double[node_num_];
    graph_bk_[i] = new double[node_num_];
    for (auto j = 0; j < node_num_; ++j) graph_[i][j] = std::numeric_limits<double>::max();
    graph_[i][i] = 0.0;
  }
  // 处理traffic routes预设节点和边
  for (auto i = 0; i < traffic_routes_.size(); ++i) {
    auto start = 2 + 2 * EXTRA_POINTS_NUM + 2 * i;
    auto end = 2 + 2 * EXTRA_POINTS_NUM + 2 * i + 1;
    auto o = std::atan2(traffic_routes_[i].b.y - traffic_routes_[i].a.y,
                        traffic_routes_[i].b.x - traffic_routes_[i].a.x);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.orientation.z = std::sin(o / 2);
    p.pose.orientation.w = std::cos(o / 2);
    p.pose.position.x = traffic_routes_[i].a.x;
    p.pose.position.y = traffic_routes_[i].a.y;
    nodes_[start] = p;
    p.pose.position.x = traffic_routes_[i].b.x;
    p.pose.position.y = traffic_routes_[i].b.y;
    nodes_[end] = p;
    // 所有traffic routes都是直线，如果支持任意曲线的话，需要计算曲线长度
    graph_[start][end] = distance(nodes_[end], nodes_[start]);
  }

  // 处理traffic routes终点能连接的起点和边
  for (auto i = 0; i < traffic_routes_.size(); ++i) {
    auto end = 2 + 2 * EXTRA_POINTS_NUM + 2 * i + 1;
    for (auto const& node : traffic_routes_[i].go_list) {
      if (node >= traffic_routes_.size()) {
        ROS_ERROR("line %d go list %d out of range", i, node);
        continue;
      }
      auto start = 2 + 2 * EXTRA_POINTS_NUM + 2 * node;
      graph_[end][start] = distance(nodes_[end], nodes_[start]);
    }
  }

  for (auto i = 0; i < node_num_; ++i) {
    for (auto j = 0; j < node_num_; ++j) {
      graph_bk_[i][j] = graph_[i][j];
    }
  }

  print_graph();
}

void HalfStructPlanner::calculate_graph() {
  struct setnode {
  point_t nearest_point;
  int line;
  double nearest_dist;

  bool operator<(setnode const& rhs) const {
    return nearest_dist < rhs.nearest_dist;
  }
  };

  std::set<setnode> start_set, goal_set;
  auto start = point_t {nodes_[0].pose.position.x, nodes_[0].pose.position.y};
  auto goal = point_t {nodes_[1].pose.position.x, nodes_[1].pose.position.y};
  setnode sn {}, gn {};
  for (auto i = 0; i < traffic_routes_.size(); ++i) {
    sn.line = i;
    sn.nearest_point = nearest_point_of_segment(start, traffic_routes_[i].a, traffic_routes_[i].b);
    if (sn.nearest_point == traffic_routes_[i].b || !line_safe(sn.nearest_point, start))
      sn.nearest_dist = std::numeric_limits<double>::max();
    else
      sn.nearest_dist = distance(start, sn.nearest_point);
    start_set.insert(sn);

    gn.line = i;
    gn.nearest_point = nearest_point_of_segment(goal, traffic_routes_[i].a, traffic_routes_[i].b);
    if (gn.nearest_point == traffic_routes_[i].a || !line_safe(gn.nearest_point, goal))
      gn.nearest_dist = std::numeric_limits<double>::max();
    else
      gn.nearest_dist = distance(goal, gn.nearest_point);
    goal_set.insert(gn);
  }

  int line_s, line_g;
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "map";
  for (auto i = 0; i < EXTRA_POINTS_NUM; ++i) {
    if (i < start_set.size()) {
      auto sen = 2 + i;
      p.pose.position.x = std::next(start_set.begin(), i)->nearest_point.x;
      p.pose.position.y = std::next(start_set.begin(), i)->nearest_point.y;
      line_g = 2 + 2 * EXTRA_POINTS_NUM + 2 * std::next(start_set.begin(), i)->line + 1;
      p.pose.orientation = nodes_[line_g].pose.orientation;
      if (i < 1 || std::next(start_set.begin(), i)->nearest_dist < EXTRA_POINTS_RANGE) {
        nodes_[sen] = p;
        graph_[0][sen] = std::next(start_set.begin(), i)->nearest_dist;
        graph_[sen][line_g] = distance(nodes_[sen], nodes_[line_g]);
      }
    }

    if (i < goal_set.size()) {
      auto gen = 2 + EXTRA_POINTS_NUM + i;
      p.pose.position.x = std::next(goal_set.begin(), i)->nearest_point.x;
      p.pose.position.y = std::next(goal_set.begin(), i)->nearest_point.y;
      line_s = 2 + 2 * EXTRA_POINTS_NUM + 2 * std::next(goal_set.begin(), i)->line;
      p.pose.orientation = nodes_[line_s].pose.orientation;
      if (i < 1 || std::next(goal_set.begin(), i)->nearest_dist < EXTRA_POINTS_RANGE) {
        nodes_[gen] = p;
        graph_[gen][1] = std::next(goal_set.begin(), i)->nearest_dist;
        graph_[line_s][gen] = distance(nodes_[line_s], nodes_[gen]);
      }
    }
  }

  // 更新起点带来的新点到终点带来的新点的代价，解决起终点在同一条line的情况
  for (auto i = 0; i < EXTRA_POINTS_NUM; ++i) {
    if (i < start_set.size()) {
      auto sen = 2 + i;
      line_s = 2 + 2 * EXTRA_POINTS_NUM + 2 * std::next(start_set.begin(), i)->line;
      line_g = line_s + 1;

      for (auto j = 0; j < EXTRA_POINTS_NUM; ++j) {
        if (j < goal_set.size()) {
          auto gen = 2 + EXTRA_POINTS_NUM + j;
          if (std::next(goal_set.begin(), j)->line == std::next(start_set.begin(), i)->line
              && distance(nodes_[gen], nodes_[line_g]) < distance(nodes_[sen], nodes_[line_g])) {
            graph_[sen][gen] = distance(nodes_[sen], nodes_[gen]);
          }
        }
      }
    }
  }

  print_graph();
//  show_graph();
}

auto HalfStructPlanner::nearest_point_of_segment(point_t const& p, point_t const& a, point_t const& b) -> point_t {
  if (a == b) return a;

  auto k = -((a.x - p.x) * (b.x - a.x) + (a.y - p.y) * (b.y - a.y)) / ((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
  if (k <= 0) return a;
  if (k >= 1) return b;
  point_t result;
  result.x = k * (b.x - a.x) + a.x;
  result.y = k * (b.y - a.y) + a.y;
  return result;
}

auto HalfStructPlanner::distance(point_t const& a, point_t const& b) -> double {
  return std::hypot(a.x - b.x, a.y - b.y);
}

void HalfStructPlanner::print_graph() {
  // 打印代价矩阵
  std::cout << "node num: " << node_num_ << std::endl;
  for (auto i = 0; i < node_num_; ++i) std::cout << GREEN << std::setw(3) << i << " " << WHITE;
  std::cout << "\n";
  for (auto i = 0; i < node_num_; ++i) {
    for (auto j = 0; j < node_num_; ++j) {
      if (graph_[i][j] != std::numeric_limits<double>::max()) {
        std::string str {"000 "};
        auto gij = std::to_string(graph_[i][j]);
        for (auto k = 0; k < 3; ++k) {
          if (gij.size() > k) str[k] = gij[k];
        }
        std::cout << str.c_str();
      } else {
        std::cout << "*** ";
      }
    }
    std::cout << GREEN << " " << i << "\n" << WHITE;
  }
}

auto HalfStructPlanner::line_safe(point_t const& a, point_t const& b) -> bool {
  if (!costmap_) return true;

  auto num = static_cast<int>(distance(a, b) / 0.05);
  uint32_t mx, my;
  double x, y;
  for (auto i = 0; i < num; ++i) {
    x = a.x + i * (b.x - a.x) / num;
    y = a.y + i * (b.y - a.y) / num;
    if (!costmap_->worldToMap(x, y, mx, my)) return false;
    if (costmap_->getCost(mx, my) >= 66) return false;
  }

  return true;
}

auto HalfStructPlanner::dijkstra() -> std::vector<int> {
  std::vector<int> result;

  struct dijnode {
  int node;
  int prev_node;
  double cost;

  bool operator<(dijnode const& rhs) const {
    return cost < rhs.cost;
  }

  bool operator==(dijnode const& rhs) const {
    return node == rhs.node;
  }
  };

  // 从graph_找到0-1的路径
  std::set<dijnode> open_list;
  std::vector<dijnode> close_list;
  open_list.insert(dijnode{0, 0, 0.0});
  while (!open_list.empty()) {
    auto node = open_list.extract(open_list.begin()).value();

    if (node.node == 1) {
      while (node.prev_node != 0) {
        result.emplace_back(node.node);
        auto prev = std::find_if(close_list.begin(), close_list.end(),
                                 [&](dijnode const& a) { return a.node == node.prev_node; });
        if (prev == close_list.end()) {
          ROS_ERROR("No whole path while graph search success, that should not happen!");
          break;
        }
        node = *prev;
      }
      result.emplace_back(node.node);
//      result.emplace_back(node.prev_node); // 不需要特意走到起点
      std::reverse(result.begin(), result.end());
      std::cout << RED << "0";
      for (auto const& r : result) std::cout << RED << "->" << r;
      std::cout << WHITE << std::endl;
      break;
    }

    close_list.emplace_back(node);

    for (auto i = 0; i < node_num_; ++i) {
      if (i == node.node) continue;
      if (std::find_if(close_list.begin(), close_list.end(),
                       [&](dijnode const& a) { return a.node == i; }) != close_list.end()) continue;
      if (graph_[node.node][i] == std::numeric_limits<double>::max()) continue;

      dijnode open_node {i, node.node, node.cost + graph_[node.node][i]};
      auto iter2 = std::find(open_list.begin(), open_list.end(), open_node);
      if (iter2 != open_list.end()) {
        if (iter2->cost > open_node.cost) {
          open_list.erase(iter2);
          open_list.insert(open_node);
        }
      } else {
        open_list.insert(open_node);
      }
    }
  }

  return result;
}
}
