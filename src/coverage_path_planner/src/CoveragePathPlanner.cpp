//
// Created by tony on 2022/10/25.
//

#include "CoveragePathPlanner.h"

namespace xju::planning {
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

CoveragePathPlanner::CoveragePathPlanner(nav_msgs::OccupancyGrid const& grid,
                                         nav_msgs::Path const& polygon,
                                         uint8_t clean_area_type) :
  initialized_(false),
  model_size_cell_(0),
  cell_free_thres_(0),
  clean_area_type_(static_cast<CleanAreaType>(clean_area_type)),
  lethal_cost_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
  weight_tiny_(WEIGHT_TINY),
  angle_(0.0),
  resolution_(0.0) {
  origin_costmap_ = grid2costmap(grid);
  read_param();
  origin_border_ = polygon_cutter(polygon);

  planning_polygon_ = origin_border_;
  angle_ = calculate_polygon_inclination(planning_polygon_, FIL_TOLER);
  on_init();
}

void CoveragePathPlanner::on_init() {
  rotate_map_polygon(angle_);
  initialized_ = initialize_mats();
}

auto CoveragePathPlanner::get_path_interface() -> std::vector<nav_msgs::Path> {
  using namespace std;

  coarse_path_.clear();
  fine_path_.clear();

  if (!initialized_) {
    ROS_ERROR("uninitialized!");
    return coarse_path_;
  }

  auto start_secs = ros::Time::now();

  auto dist_threshold = AMPLIFY * resolution_ * model_size_cell_;
  dist_threshold = dist_threshold * dist_threshold;

  nav_msgs::Path path_pool;
  geometry_msgs::PoseStamped posestamped_pool;
  geometry_msgs::Pose pose_pool;

  std::optional<std::vector<CellIndex>> cells;
  cells = (clean_area_type_ == CleanAreaType::backshaped)
          ? get_backshaped_indexed_path()
          : get_zigzag_indexed_path();
  if (!cells) return coarse_path_;

  /**transform**/
  auto size_y = rotated_costmap_->getSizeInCellsY();
  for (auto const& cell : cells.value()) {
    rotated_costmap_->mapToWorld(static_cast<uint32_t>(cell.col * model_size_cell_
                                                       + cells_poses_[cell.row][cell.col].col),
                                 static_cast<uint32_t>(size_y - (cell.row * model_size_cell_
                                                                 + cells_poses_[cell.row][cell.col].row) - 1),
                                 pose_pool.position.x,
                                 pose_pool.position.y);

    pose_pool.orientation.w = 1;
    pose_pool.orientation.x = 0;
    pose_pool.orientation.y = 0;
    pose_pool.orientation.z = 0;
    posestamped_pool.header.stamp = ros::Time::now();
    posestamped_pool.header.frame_id = "map";
    posestamped_pool.pose = pose_pool;

    if (!path_pool.poses.empty()) {
      if ((dist2(posestamped_pool, path_pool.poses.back())) > dist_threshold) {
        coarse_path_.emplace_back(path_pool);
        path_pool.poses.clear();
        path_pool.poses.emplace_back(posestamped_pool);
        continue;
      }

      path_pool.poses.emplace_back(posestamped_pool);
    } else {
      path_pool.poses.emplace_back(posestamped_pool);
    }
  }

  coarse_path_.emplace_back(path_pool);

#ifdef DEBUG_MODE
  ofstream f1(debug_path_ + "origin.txt", ios::trunc | ios::out);
  for (auto const& pose : planning_polygon_.poses) {
      f1 << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.orientation.z << endl;
  }
#endif
#ifdef DEBUG_MODE
#ifdef REVERSE_COARSE_PATH
  std::reverse(coarse_path_.begin(), coarse_path_.end());
  for (auto& path: coarse_path_) {
      std::reverse(path.poses.begin(), path.poses.end());
  }
#endif
#endif
  for (auto& path: coarse_path_) {
    auto iter = path.poses.begin();
    for (; iter < path.poses.end() - 1; iter++) {
      (*iter).pose.orientation.z = atan2((*(iter + 1)).pose.position.y - (*iter).pose.position.y,
                                         (*(iter + 1)).pose.position.x - (*iter).pose.position.x);
#ifdef DEBUG_MODE
      f1 << (*iter).pose.position.x << "," << (*iter).pose.position.y << "," << (*iter).pose.orientation.z << endl;
#endif
    }
    (*iter).pose.orientation.z = (*(iter - 1)).pose.orientation.z;
#ifdef DEBUG_MODE
    f1 << (*iter).pose.position.x << "," << (*iter).pose.position.y << "," << (*iter).pose.orientation.z << endl;
#endif
  }
#ifdef DEBUG_MODE
  f1.close();
#endif

  /**cut down way points**/
  for (auto& path: coarse_path_) {
    path.poses.back().pose.orientation.x = static_cast<double>(CoarsePathSeparate::break_point);   // 每条路径的终点都应当标记为break point
    auto path_length = path.poses.size();
    for (auto it = path.poses.begin() + 1; it < path.poses.end() - 1; it++) {
      // #348 opposite direction
      if (abs(abs((it - 1)->pose.orientation.z - it->pose.orientation.z) - M_PI) < ZERO
          && it->pose.orientation.z == (it + 1)->pose.orientation.z) {
        (it - 1)->pose.orientation.x = static_cast<double>(CoarsePathSeparate::delete_point);
        (it + 1)->pose.orientation.x = static_cast<double>(CoarsePathSeparate::break_point);
      }

      if ((it - 1)->pose.orientation.z == it->pose.orientation.z
          && it->pose.orientation.z == (it + 1)->pose.orientation.z
          && it->pose.orientation.x != static_cast<double>(CoarsePathSeparate::break_point)) { //SS
        it->pose.orientation.x = static_cast<double>(CoarsePathSeparate::delete_point);
      }

      if (it >= path.poses.begin() + 2
          && (it - 2)->pose.orientation.z != (it - 1)->pose.orientation.z
          && (it - 1)->pose.orientation.z == it->pose.orientation.z
          && it->pose.orientation.z == (it + 1)->pose.orientation.z) { // CSS
        it->pose.orientation.x = static_cast<double>(CoarsePathSeparate::break_point);
      }

      if (it >= path.poses.begin() + 2
          && (it - 2)->pose.orientation.z == (it - 1)->pose.orientation.z
          && (it - 1)->pose.orientation.z == it->pose.orientation.z
          && it->pose.orientation.z != (it + 1)->pose.orientation.z) { // SSC
        it->pose.orientation.x = static_cast<double>(CoarsePathSeparate::break_point);
      }

      if (path_length > 5
          && it >= path.poses.begin() + 2
          && (it - 1)->pose.orientation.z != it->pose.orientation.z
          && (it - 2)->pose.orientation.x != static_cast<double>(CoarsePathSeparate::break_point)) {
        (it - 1)->pose.orientation.x = static_cast<double>(CoarsePathSeparate::break_point);
      }
    }
  }

  nav_msgs::Path path_temp;
  vector<nav_msgs::Path> paths_temp;
  for (auto& path: coarse_path_) {
    for (auto& pose: path.poses) {
      if (pose.pose.orientation.x != static_cast<double>(CoarsePathSeparate::delete_point)) {
        path_temp.poses.emplace_back(pose);
      }
      if (pose.pose.orientation.x == static_cast<double>(CoarsePathSeparate::break_point)) {
        paths_temp.emplace_back(path_temp);
        path_temp.poses.clear();
        path_temp.poses.emplace_back(pose);
      }
    }
    paths_temp.emplace_back(path_temp);
    path_temp.poses.clear();
  }

//	    auto cv_secs = ros::Time::now();
//	    ROS_WARN("Get way points time is %.2f secs", (cv_secs - start_secs).toSec());

#ifdef DEBUG_MODE
  ofstream f2(debug_path_ + "cutdown.txt", ios::trunc | ios::out);
  for (auto const& path: paths_temp) {
      for (auto const& pose: path.poses) {
          f2 << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.orientation.z << endl;
      }
  }
  f2.close();
#endif

  /**rotate back path**/
  if (angle_ != 0.0) {
#ifdef DEBUG_MODE
    ofstream f3(debug_path_ + "rotateback.txt", ios::trunc | ios::out);
#endif
    auto cos_an = cos(-angle_);
    auto sin_an = sin(-angle_);
    for (auto& path: paths_temp) {
      for (auto& pose: path.poses) {
        auto temp_x = pose.pose.position.x;
        auto temp_y = pose.pose.position.y;
        pose.pose.position.x = center_.x + (temp_x - center_.x) * cos_an - (temp_y - center_.y) * sin_an;
        pose.pose.position.y = center_.y + (temp_x - center_.x) * sin_an + (temp_y - center_.y) * cos_an;
#ifdef DEBUG_MODE
        f3 << pose.pose.position.x << "," << pose.pose.position.y << endl;
#endif
      }
    }
#ifdef DEBUG_MODE
    f3.close();
#endif
  }
//	    auto rotate_secs = ros::Time::now();
//	    ROS_WARN("Rotate way points time is %.2f secs", (rotate_secs - cv_secs).toSec());

  /**optimize**/
  vector<nav_msgs::Path> paths_opt_temp;
  for (size_t i = 0; i < paths_temp.size(); i++) {
    auto om_path = get_bezier_optimized_path(paths_temp[i]);
    if (om_path.poses.size() < 2) continue;
    if (i != paths_temp.size() - 1) {
      om_path.poses.pop_back();
    }

    paths_opt_temp.emplace_back(om_path);
  }

  path_temp.poses.clear();

  nav_msgs::Path border;
  bool border_contained;
  if (save_border(border)) {
    ROS_INFO("Save border.");
    border_contained = true;
    fine_path_.emplace_back(border);
  } else {
    ROS_WARN("Rect non free");
    border_contained = false;
  }

  adjust_border(paths_temp.front(), border);

  if (!paths_opt_temp.empty()) {
    auto iter = paths_opt_temp.begin();
    for (; iter < paths_opt_temp.end() - 1; iter++) {
      path_temp.poses.insert(path_temp.poses.end(), (*iter).poses.begin(), (*iter).poses.end());
      if (dist2((*iter).poses.back(), (*(iter + 1)).poses.front()) > 0.1) {
        fine_path_.emplace_back(path_temp);
        path_temp.poses.clear();
      }
    }
    path_temp.poses.insert(path_temp.poses.end(), (*iter).poses.begin(), (*iter).poses.end());
  }
  fine_path_.emplace_back(path_temp);

#ifdef DEBUG_MODE
  ofstream f4(debug_path_ + "optimize.txt", ios::trunc | ios::out);
  for (auto const& path : fine_path_) {
      for (auto const& pose : path.poses) {
          f4 << pose.pose.position.x << "," << pose.pose.position.y << endl;
      }
  }

  f4.close();
#endif
  auto iter = fine_path_.begin();
  if (border_contained) ++iter;
  for (; iter != fine_path_.end(); ++iter) {
    for (auto& pose: (*iter).poses) {
      if (pose.pose.orientation.x != 0) {
        pose.pose.orientation.x = 0;
      }
    }
  }
  // #479 work around: check abnormal points
  iter = fine_path_.begin();
  if (border_contained) {
    ++iter;
  }

  for (; iter != fine_path_.end(); ++iter) {
    for (auto it = (*iter).poses.begin(); it != (*iter).poses.end();) {
      if (abs((*it).pose.position.x) < ZERO
          && abs((*it).pose.position.y) < ZERO
          && abs((*it).pose.orientation.z) < ZERO
          && abs((*it).pose.orientation.w - 1.0) < ZERO) {
        ROS_ERROR_STREAM("Found abnormal zero points!");
        it = (*iter).poses.erase(it);
      } else {
        ++it;
      }
    }
  }

  auto optimize_secs = ros::Time::now();
//	    ROS_WARN("Optimize way points time is %.2f secs", (optimize_secs - rotate_secs).toSec());
  ROS_WARN("Total time is %.2f secs", (optimize_secs - start_secs).toSec());
  ROS_INFO("Generate %lu paths.", fine_path_.size());

  return fine_path_;
}

auto CoveragePathPlanner::initialize_mats() -> bool {
  rotated_mat_ = costmap2cv_mat(rotated_costmap_);
  if (rotated_mat_.empty()) return false;
  cells_poses_ = get_free_cell_in_polygon(rotated_mat_, scale_mat_, free_space_);

  neural_mat_ = cv::Mat(scale_mat_.rows, scale_mat_.cols, CV_32F);
  initialize_neural_mat(scale_mat_, neural_mat_);
//        imwrite(debug_path_ + "debug_srcmap.jpg", rotated_mat_);
  return !neural_mat_.empty();
}

void CoveragePathPlanner::initialize_neural_mat(cv::Mat const& src, cv::Mat& des) const {
  for (auto i = 0; i < des.rows; ++i) {
    for (auto j = 0; j < des.cols; ++j) {
      if (src.at<uchar>(i, j) == costmap_2d::LETHAL_OBSTACLE) {
        des.at<float>(i, j) = -FLT_MAX;
      } else {
        des.at<float>(i, j) = C_0 - static_cast<float>(j) / static_cast<float>(des.cols);
      }
    }
  }
}

auto CoveragePathPlanner::get_zigzag_indexed_path() -> std::optional<std::vector<CellIndex>> {
  if (free_space_.empty()) {
    ROS_ERROR("No free space!");
    return std::nullopt;
  }

  if (planning_polygon_.poses.empty()) {
    ROS_ERROR("No zone peak!");
    return std::nullopt;
  }

  auto ret_tuple = get_start_point(planning_polygon_);
  if (!ret_tuple) {
    ROS_ERROR("Failed to set initial pose in map!");
    return std::nullopt;
  }

  using namespace std;

  std::vector<CellIndex> indexed_path;
  CellIndex init_point, next_point, cur_point;
  init_point.theta = 0;

  auto wx = std::get<0>(ret_tuple.value());
  auto wy = std::get<1>(ret_tuple.value());
  auto mx = std::get<2>(ret_tuple.value());
  auto my = std::get<3>(ret_tuple.value());
  ROS_INFO("start inittheta is %.2f, start point is [%.4f, %.4f]", init_point.theta, wx, wy);

  change_weight_tiny(my);
  init_point.row = std::get<4>(ret_tuple.value()).row;
  init_point.col = std::get<4>(ret_tuple.value()).col;
  cur_point = init_point;
  //处理起点在右侧或者右侧有障碍物的情况
  cur_point.theta = (cur_point.col == neural_mat_.cols - 1
                     || neural_mat_.at<float>(cur_point.row, cur_point.col + 1) == -FLT_MAX)
                    ? 180 : 0;
  indexed_path.emplace_back(init_point);
  //将第一个点插入indexed_path之后，就从遍历集中删除
  auto free_space = free_space_;
  auto init_point_ptr = find_if(free_space.begin(),
                                free_space.end(),
                                [&](CellIndex const& cell) {
                                  return cell.col == init_point.col && cell.row == init_point.row;
                                });
  if (init_point_ptr != free_space.end()) {
    free_space.erase(init_point_ptr);
  }

  float e = 0;
  float v = 0.0;
  float delta_theta = 0;
  vector<float> thetaVec = {0, 180};
  /**main planning loop**/
  auto bound_of_free_space = FreeSpaceCalculator::find_bound_of_free_space(free_space_, neural_mat_);
  auto min_row_of_free_space = std::get<1>(bound_of_free_space);
  auto min_col_of_free_space = std::get<0>(bound_of_free_space);
  for (int loop = 0; loop < LOOP_MAX; ++loop) {
    auto maxIndex = 0UL;
    float max_v = OUT_SCORE - 1;
    neural_mat_.at<float>(cur_point.row, cur_point.col) = OUT_SCORE;
    auto th = static_cast<float>(cur_point.theta);
    for (size_t id = 0; id < thetaVec.size(); ++id) {
      delta_theta = std::abs(thetaVec[id] - th);
      if (delta_theta > 180) {
        delta_theta = 360 - delta_theta;
      }

      e = 1 - abs(delta_theta) / 180;
      switch (id) {
        case 0:
          if (cur_point.col == neural_mat_.cols - 1) {
            v = -FLT_MAX;
            break;
          }

          v = neural_mat_.at<float>(cur_point.row, cur_point.col + 1) + C_0 * e;
          break;
        case 1:
          if (cur_point.col == 0) {
            v = -FLT_MAX;
            break;
          }

          v = neural_mat_.at<float>(cur_point.row, cur_point.col - 1) + C_0 * e + weight_tiny_;
          break;
        default:
          break;
      }

      if (v > max_v || (v == max_v && id > maxIndex)) {
        max_v = v;
        maxIndex = id;
      }
    }

    if (free_space.empty()) {
      ROS_WARN("The program has gone through %lu steps", indexed_path.size());
      return indexed_path;
    }

    if (max_v <= 0) {
      float min_dist = FLT_MAX;
      float dist;
      int ii = 0;
      int min_index = -1;
      for (auto const& fs : free_space_) {
        if (neural_mat_.at<float>(fs.row, fs.col) > 0) {
          dist = static_cast<float>((cur_point.row - fs.row) * (cur_point.row - fs.row)
                                    + (cur_point.col - fs.col) * (cur_point.col - fs.col));
          if (dist < min_dist) {
            min_dist = dist;
            min_index = ii;
          }
        }

        ++ii;
      }

      if (min_index == -1) {
        ROS_WARN("The program has gone through %lu steps", indexed_path.size());
        break;
      }

      next_point = free_space_[static_cast<unsigned long>(min_index)];
      next_point.theta = atan2(next_point.col - cur_point.col, next_point.row - cur_point.row) / M_PI * 180;
      if (next_point.theta < 0) {
        next_point.theta = 360 + next_point.theta;
      }

      ROS_DEBUG_STREAM("next point index: " << min_index << " " << "distance: " << std::sqrt(min_dist));
      ROS_DEBUG_STREAM("current point: " << cur_point.row << ", " << cur_point.col);
      ROS_DEBUG_STREAM("next point: " << next_point.row << ", " << next_point.col);
      cur_point = next_point;
      indexed_path.emplace_back(next_point);

      continue;
    }


    /**next point**/
    switch (maxIndex) {
      case 0:
        next_point.row = cur_point.row;
        next_point.col = cur_point.col + 1;
        break;
      case 1:
        next_point.row = cur_point.row;
        next_point.col = cur_point.col - 1;
        break;
      default:
        break;
    }

    ROS_INFO("in same line");
    ROS_INFO_STREAM("current point: " << cur_point.row << ", " << cur_point.col);
    ROS_INFO_STREAM("next point: " << next_point.row << ", " << next_point.col);
    next_point.theta = thetaVec[maxIndex];
    cur_point = next_point;
    indexed_path.emplace_back(next_point);
    auto next_ptr = find_if(free_space.begin(),
                            free_space.end(),
                            [&](CellIndex const& cell) {
                              return cell.row == next_point.row && cell.col == next_point.col;
                            });
    if (next_ptr != free_space.end()) {
      free_space.erase(next_ptr);
    }
  }

  ROS_WARN("CoveragePathPlanner: in function get_zigzag_indexed_path, loop reaches max(%d)", LOOP_MAX);
  return indexed_path;
}

auto CoveragePathPlanner::get_backshaped_indexed_path() -> std::optional<std::vector<CellIndex>> {
  if (free_space_.empty()) {
    ROS_ERROR("No free space!");
    return std::nullopt;
  }

  if (planning_polygon_.poses.empty()) {
    ROS_ERROR("No zone peak!");
    return std::nullopt;
  }

  auto ret_tuple = get_start_point(planning_polygon_);
  if (!ret_tuple) {
    ROS_ERROR("Failed to set initial pose in map!");
    return std::nullopt;
  }

  std::vector<CellIndex> indexed_path;
  CellIndex init_point, next_point, cur_point;

  auto wx = std::get<0>(ret_tuple.value());
  auto wy = std::get<1>(ret_tuple.value());
  auto mx = std::get<2>(ret_tuple.value());
  auto my = std::get<3>(ret_tuple.value());
  change_weight_tiny(my);

  using dir_t = std::tuple<int, int, int>;
  std::vector<dir_t> directs{
    std::make_tuple(0, 0, 1),
    std::make_tuple(1, -1, 1),
    std::make_tuple(2, -1, 0),
    std::make_tuple(3, -1, -1),
    std::make_tuple(4, 0, -1),
    std::make_tuple(5, 1, -1),
    std::make_tuple(6, 1, 0),
    std::make_tuple(7, 1, 1)};
  auto closewise = is_clockwise(origin_border_);
  if (closewise) {
    std::reverse(directs.begin(), directs.end());
  }

  init_point.row = scale_mat_.rows - static_cast<int32_t>(my) / model_size_cell_ - 1;
  init_point.col = static_cast<int32_t>(mx) / model_size_cell_;
  init_point.theta = closewise ? 180 : 0;
  indexed_path.emplace_back(init_point);
  ROS_INFO("Init theta is %.2f, start point is [%.4f, %.4f]", init_point.theta, wx, wy);

  cur_point = init_point;
  auto current_theta = static_cast<int>(init_point.theta / 45);

  while (ros::ok()) {
    neural_mat_.at<float>(cur_point.row, cur_point.col) = OUT_SCORE;
    auto rotate_dist = std::distance(directs.begin(),
                                     std::find_if(directs.begin(), directs.end(),
                                                  [&](dir_t const& a) {
                                                    return current_theta == std::get<0>(a);
                                                  })) - static_cast<int>(directs.size() / 2);
    rotate_dist = (rotate_dist + static_cast<int>(directs.size())) % static_cast<int>(directs.size());
    std::rotate(directs.begin(), directs.begin() + rotate_dist, directs.end());

    auto find_near = false;
    for (auto const& dir : directs) {
      auto r = cur_point.row + std::get<1>(dir);
      auto c = cur_point.col + std::get<2>(dir);
      if (std::max(0, r) == std::min(r, neural_mat_.rows - 1)
          && std::max(0, c) == std::min(c, neural_mat_.cols - 1)
          && neural_mat_.at<float>(r, c) > 0) {
        find_near = true;
        next_point.row = r;
        next_point.col = c;
        next_point.theta = std::get<0>(dir) * 45;
        indexed_path.emplace_back(next_point);
        cur_point = next_point;
        current_theta = std::get<0>(dir);
        break;
      }
    }

    if (find_near) continue;

    float min_dist = FLT_MAX;
    float dist;
    int ii = 0;
    int min_index = -1;
    for (auto const& fs : free_space_) {
      if (neural_mat_.at<float>(fs.row, fs.col) > 0) {
        dist = static_cast<float>((cur_point.row - fs.row) * (cur_point.row - fs.row)
                                  + (cur_point.col - fs.col) * (cur_point.col - fs.col));
        if (dist < min_dist) {
          min_dist = dist;
          min_index = ii;
        }
      }

      ++ii;
    }

    if (min_index == -1) {
      ROS_WARN("The program has gone through %lu steps", indexed_path.size());
      break;
    }

    next_point = free_space_[static_cast<unsigned long>(min_index)];
    next_point.theta = atan2(next_point.col - cur_point.col, next_point.row - cur_point.row) / M_PI * 180;
    if (next_point.theta < 0) {
      next_point.theta = 360 + next_point.theta;
    }

    ROS_DEBUG_STREAM("next point index: " << min_index << " " << "distance: " << std::sqrt(min_dist));
    ROS_DEBUG_STREAM("current point: " << cur_point.row << ", " << cur_point.col);
    ROS_DEBUG_STREAM("next point: " << next_point.row << ", " << next_point.col);
    cur_point = next_point;
    indexed_path.emplace_back(next_point);
    current_theta = closewise ? 4 : 0;
  }

  return indexed_path;
}

auto CoveragePathPlanner::get_bezier_optimized_path(nav_msgs::Path const& path) const -> nav_msgs::Path {
  BezierTrajectoryGeneratorWaypoint bezier_traj_gen_way_points(PATH_STEP);
  return bezier_traj_gen_way_points.calculate_optimized_path(path);
}

void CoveragePathPlanner::adjust_border(nav_msgs::Path const& cover, nav_msgs::Path& border) {
  using namespace std;

  auto dist2_min = DBL_MAX;
  size_t idx = 0;
  for (size_t i = 0; i < border.poses.size(); ++i) {
    auto dist2_tmp = dist2(border_polygon_.poses[i], cover.poses.front());
    if (dist2_tmp < dist2_min) {
      dist2_min = dist2_tmp;
      idx = i;
    }
  }

  if (idx != 0) {
    vector<geometry_msgs::PoseStamped> fb, bb;
    fb.insert(fb.begin(), border.poses.begin(), border.poses.begin() + static_cast<long>(idx));
    bb.insert(bb.begin(), border.poses.begin() + static_cast<long>(idx), border.poses.end());
    border.poses.clear();
    border.poses.insert(border.poses.end(), bb.begin(), bb.end());
    border.poses.insert(border.poses.end(), fb.begin(), fb.end());
  }
}

auto CoveragePathPlanner::save_border(nav_msgs::Path& border) -> bool {
  if (border_polygon_.poses.size() < 2) return false;
  std::vector<nav_msgs::Path> border_tmp;
  for (size_t i = 0; i < border_polygon_.poses.size(); ++i) {
    nav_msgs::Path path_tmp;
    path_tmp.poses.emplace_back(border_polygon_.poses[i]);
    if (i == border_polygon_.poses.size() - 1) {
      path_tmp.poses.emplace_back(border_polygon_.poses.front());
    } else {
      path_tmp.poses.emplace_back(border_polygon_.poses[i + 1]);
    }
    border_tmp.emplace_back(path_tmp);
  }

  for (auto const& tmp : border_tmp) {
    auto om_path = get_bezier_optimized_path(tmp);
    om_path.poses.pop_back();
    border.poses.insert(border.poses.end(), om_path.poses.begin(), om_path.poses.end());
  }

  if (border_polygon_.poses.size() > 4) return true;

  int non_free = 0;
  for (auto const& pose : border.poses) {
    auto x = pose.pose.position.x;
    auto y = pose.pose.position.y;
    unsigned mx, my;
    if (!origin_costmap_->worldToMap(x, y, mx, my)) {
      if (++non_free > 3) return false;
      continue;
    }

    if (origin_costmap_->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      if (++non_free > 3) return false;
    }
  }

  return true;
}

auto CoveragePathPlanner::get_free_cell_in_polygon(cv::Mat const& src,
                                                   cv::Mat& des,
                                                   std::vector<CellIndex>& free_space)
-> std::vector<std::vector<SimpleCellIndex>> {
  auto free_space_calculator = std::make_unique<FreeSpaceCalculator>(cell_free_thres_,
                                                                     model_size_cell_,
                                                                     lethal_cost_);
  free_space_calculator->get_free_cell_in_polygon(src,
                                                  des,
                                                  free_space,
                                                  planning_polygon_,
                                                  rotated_costmap_,
                                                  true);
  return free_space_calculator->get_cells_poses();
}

auto CoveragePathPlanner::get_start_point(nav_msgs::Path const& polygon)
-> std::optional<std::tuple<double, double, uint32_t, uint32_t, CellIndex>> {
  size_t min_index = 0;
  auto x = planning_polygon_.poses[min_index].pose.position.x;
  auto y = planning_polygon_.poses[min_index].pose.position.y;
  unsigned mx, my;
  if (!rotated_costmap_->worldToMap(x, y, mx, my)) {
    return std::nullopt;
  }

  auto r0 = scale_mat_.rows - static_cast<int32_t>(my) / model_size_cell_ - 1;
  auto c0 = static_cast<int32_t>(mx) / model_size_cell_;
  int min_dist = INT_MAX;
  auto r_min = 0;
  auto c_min = 0;
  for (auto const& cell: free_space_) {
    if ((abs(cell.row - r0) + abs(cell.col - c0)) < min_dist) {
      min_dist = abs(cell.row - r0) + abs(cell.col - c0);
      r_min = cell.row;
      c_min = cell.col;
    }
  }

  mx = static_cast<uint32_t>(c_min * model_size_cell_ + model_size_cell_ / 2);
  my = static_cast<uint32_t>((scale_mat_.rows - r_min - 1) * model_size_cell_ + model_size_cell_ / 2);
  rotated_costmap_->mapToWorld(mx, my, x, y);
  return std::make_tuple(x, y, mx, my, CellIndex{r_min, c_min, 0});
}

auto CoveragePathPlanner::calculate_polygon_inclination(nav_msgs::Path const& polygon, double tolerance) -> double {
  auto angle = 0.0;
  /**rotate**/
  if (polygon.poses.size() < 3) {
    angle = 0.0;
  } else {
    std::vector<cv::Point2f> cv_polygon;
    for (auto const& pose : polygon.poses) {
      cv_polygon.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    auto longest = 0.0;
    auto rect = cv::minAreaRect(cv_polygon);
    if (rect.size.height > rect.size.width) {
      longest = rect.size.height;
      angle = -(90 + rect.angle) / 180.0 * M_PI;
    } else {
      longest = rect.size.width;
      angle = -rect.angle / 180.0 * M_PI;
    }

    ROS_INFO("CoveragePathPlanner: Find longest edge = %.2f", longest);
    ROS_INFO("CoveragePathPlanner: Angle = %.2f deg, start idx = %d", angle * 180.0 / M_PI, 0);
  }

  return angle;
}

auto CoveragePathPlanner::polygon_cutter(nav_msgs::Path const& polygon) const -> nav_msgs::Path {
  nav_msgs::Path cut_polygon = polygon;

  /**Revise the Goal**/
  auto check_dist2 = CHECK_DIST * CHECK_DIST;
  if (dist2(cut_polygon.poses.back(), cut_polygon.poses.front()) < check_dist2
      && cut_polygon.poses.size() > CHECK_MAX) {
    auto dist2_min = FLT_MAX;
    auto iter = cut_polygon.poses.end();
    for (auto it = cut_polygon.poses.end() - CHECK_MAX; it != cut_polygon.poses.end(); ++it) {
      auto dist2_tmp = dist2((*it), cut_polygon.poses.front());
      if (dist2_tmp > check_dist2)
        continue;
      if (dist2_min > dist2_tmp) {
        dist2_min = static_cast<float>(dist2_tmp);
        iter = it;
      }
    }

    cut_polygon.poses.erase(iter, cut_polygon.poses.end());
  }

  return cut_polygon;
}

void CoveragePathPlanner::rotate_map_polygon(double angle) {
  border_polygon_ = planning_polygon_;
  resolution_ = origin_costmap_->getResolution();
  if (0.0 == angle) {
    rotated_costmap_ = origin_costmap_;
    ROS_DEBUG_STREAM("polygon angle is 0, do not need rotate");
    return;
  }

  ROS_DEBUG_STREAM("before rotate, planning polygon is");
  for (auto const& pose: planning_polygon_.poses) {
    ROS_DEBUG_STREAM("[" << pose.pose.position.x << ", " << pose.pose.position.y << "]");
  }

  auto cos_p = cos(angle);
  auto sin_p = sin(angle);
  auto cos_n = cos_p;
  auto sin_n = -sin_p;
  ROS_INFO("angle: %.2f, cos: %.2f, sin: %.2f", angle, cos_p, sin_p);

  auto width_cell_old = origin_costmap_->getSizeInCellsX();
  auto height_cell_old = origin_costmap_->getSizeInCellsY();
  auto width_cell = (unsigned) (abs(width_cell_old * cos_p) + abs(height_cell_old * sin_p));
  auto height_cell = (unsigned) (abs(width_cell_old * sin_p) + abs(height_cell_old * cos_p));
  auto width_old = width_cell_old * origin_costmap_->getResolution();
  auto height_old = height_cell_old * origin_costmap_->getResolution();
  auto width = width_cell * origin_costmap_->getResolution();
  auto height = height_cell * origin_costmap_->getResolution();
  ROS_INFO("old costmap size: (%.2f, %.2f); new costmap size: (%.2f, %.2f)", width_old, height_old, width, height);

  center_.x = origin_costmap_->getOriginX() + width_old / 2;
  center_.y = origin_costmap_->getOriginY() + height_old / 2;

  auto origin_x = center_.x - width / 2;
  auto origin_y = center_.y - height / 2;
  ROS_INFO("both center: (%.2f, %.2f); old origin: (%.2f, %.2f); new origin: (%.2f, %.2f)",
           center_.x, center_.y, origin_costmap_->getOriginX(), origin_costmap_->getOriginY(), origin_x, origin_y);

  rotated_costmap_ = std::make_shared<costmap_2d::Costmap2D>(width_cell,
                                                             height_cell,
                                                             resolution_,
                                                             origin_x,
                                                             origin_y);

  for (unsigned i = 0; i < width_cell; i++) {
    for (unsigned j = 0; j < height_cell; j++) {
      double wx, wy;
      rotated_costmap_->mapToWorld(i, j, wx, wy);
      auto wx_old = center_.x + (wx - center_.x) * cos_n - (wy - center_.y) * sin_n;
      auto wy_old = center_.y + (wx - center_.x) * sin_n + (wy - center_.y) * cos_n;
      if (wx_old < origin_costmap_->getOriginX() || wx_old > origin_costmap_->getOriginX() + width_old ||
          wy_old < origin_costmap_->getOriginY() || wy_old > origin_costmap_->getOriginY() + height_old) {
        rotated_costmap_->setCost(i, j, costmap_2d::NO_INFORMATION);
      } else {
        unsigned ii, jj;
        origin_costmap_->worldToMap(wx_old, wy_old, ii, jj);
        rotated_costmap_->setCost(i, j, origin_costmap_->getCost(ii, jj));
      }
    }
  }
#ifdef DEBUG_MODE
  rotated_costmap_->saveMap(debug_path_ + "rotatedfunc.pgm");
#endif
  ROS_INFO("Rotate info: center: (%.2f, %.2f)", center_.x, center_.y);
  for (auto& pose: planning_polygon_.poses) {
    auto temp_x = pose.pose.position.x;
    auto temp_y = pose.pose.position.y;
    pose.pose.position.x = center_.x + (temp_x - center_.x) * cos_p - (temp_y - center_.y) * sin_p;
    pose.pose.position.y = center_.y + (temp_x - center_.x) * sin_p + (temp_y - center_.y) * cos_p;
  }
//        rotated_costmap_->saveMap(debug_path_ + "rotated.pgm");
  ROS_DEBUG_STREAM("after rotate, planning polygon is");
  for (auto const& pose: planning_polygon_.poses) {
    ROS_DEBUG_STREAM("[" << pose.pose.position.x << ", " << pose.pose.position.y << "]");
  }

  ROS_INFO("The size of map is [%d, %d] with resolution %.2f",
           rotated_costmap_->getSizeInCellsX(),
           rotated_costmap_->getSizeInCellsY(),
           resolution_);
}

auto
CoveragePathPlanner::grid2costmap(nav_msgs::OccupancyGrid const& grid) const -> std::shared_ptr<costmap_2d::Costmap2D> {
  auto costmap = std::make_shared<costmap_2d::Costmap2D>(grid.info.width,
                                                         grid.info.height,
                                                         grid.info.resolution,
                                                         grid.info.origin.position.x,
                                                         grid.info.origin.position.y);
  costmap->setDefaultValue(255);

  costmap->resetMap(0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  for (unsigned y = 0; y < grid.info.height; y++) {
    for (unsigned x = 0; x < grid.info.width; x++) {
      if (grid.data[y * grid.info.width + x] < 0) {
        costmap->setCost(x, y, costmap_2d::NO_INFORMATION);
        continue;
      }

      costmap->setCost(x, y, static_cast<uint8_t>(grid.data[y * grid.info.width + x] * 254 / 100));
    }
  }

  return costmap;
}

auto CoveragePathPlanner::costmap2cv_mat(std::shared_ptr<costmap_2d::Costmap2D> const& costmap,
                                         int32_t x,
                                         int32_t y,
                                         int32_t width,
                                         int32_t height) -> cv::Mat {
  auto ret_mat = cv::Mat(height, width, CV_8U);
  for (auto r = 0; r < height; r++) {
    for (auto c = 0; c < width; c++) {
      //costmap's origin is at left bottom ,while opencv's pic's origin is at left-top.
      auto mx = c + x;
      auto my = y + height - r - 1;
      if (mx < 0 || mx >= costmap->getSizeInCellsX() || my < 0 || my >= costmap->getSizeInCellsY()) {
        continue;
      }

      ret_mat.at<uchar>(r, c)
        = costmap->getCost(static_cast<unsigned>(mx), static_cast<unsigned>(my));
    }
  }

  return ret_mat;
}

void CoveragePathPlanner::change_weight_tiny(uint32_t first_y) {
  // renew weight_tiny_
  uint32_t mx_poly_center, my_poly_center;
  rotated_costmap_->worldToMap(polygon_center_.x, polygon_center_.y, mx_poly_center, my_poly_center);
  weight_tiny_ = my_poly_center > first_y ? WEIGHT_TINY : -WEIGHT_TINY;
#ifdef DEBUG_MODE
  ROS_ERROR("y of planning_polygon_ in rotated_costmap_ = %u", first_y);
  ROS_ERROR("center of polygon in rotated_costmap_ = [%u, %u]", mx_poly_center, my_poly_center);
#endif
}

auto CoveragePathPlanner::border_filter(nav_msgs::Path const& path) -> nav_msgs::Path {
  uint32_t last_mx, last_my;
  origin_costmap_->worldToMap(
    path.poses.front().pose.position.x,
    path.poses.front().pose.position.y,
    last_mx,
    last_my);
  uint32_t last_mx_save = last_mx;
  uint32_t last_my_save = last_my;
  uint32_t mx, my;
  nav_msgs::Path temp_path;
  temp_path.poses.emplace_back(path.poses.front());
  for (auto const& pose : path.poses) {
    origin_costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
    if ((mx != last_mx || my != last_my)) {
      temp_path.poses.emplace_back(pose);
      last_mx = mx;
      last_my = my;
    }
  }

  if (last_mx == last_mx_save && last_my == last_my_save) {
    temp_path.poses.pop_back();
  }

#ifdef DEBUG_MODE
  ROS_INFO("after border_filter, %lu points left:", temp_path.poses.size());
  for (auto const& pose : temp_path.poses) {
      ROS_INFO("[%.2f, %.2f]", pose.pose.position.x, pose.pose.position.y);
  }
#endif

  nav_msgs::Path ret_path = border_direction_filter(temp_path);

#ifdef DEBUG_MODE
  ROS_INFO("after border_direction_filter, %lu points left:", ret_path.poses.size());
  for (auto const& pose : ret_path.poses) {
      ROS_INFO("[%.2f, %.2f]", pose.pose.position.x, pose.pose.position.y);
  }
#endif
  return ret_path;
}

auto CoveragePathPlanner::border_direction_filter(nav_msgs::Path& path) -> nav_msgs::Path {
  BezierTrajectoryGeneratorWaypoint::calculate_path_orientation(path, false);
  return path;
}

void CoveragePathPlanner::read_param() {
  ros::NodeHandle private_nh("~");
  private_nh.param<int>("size_of_cell", model_size_cell_, 3);
  private_nh.param<int>("free_threshold", cell_free_thres_, 250);

  auto temp_lethal_cost = 0;
  private_nh.param<int32_t>("WallFollowing/lethal_cost",
                            temp_lethal_cost,
                            static_cast<int32_t>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
  lethal_cost_ = static_cast<uint8_t>(temp_lethal_cost);
  private_nh.param<std::string>("debug_path", debug_path_, std::string(""));
}

auto CoveragePathPlanner::is_clockwise(nav_msgs::Path const& path) -> bool {
  auto n = path.poses.size();
  if (n < 3) return false;

  int i, j, k;
  int count = 0;
  double z;
  for (i = 0; i < n; ++i) {
    j = (i + 1) % n;
    k = (i + 2) % n;
    z = (path.poses[j].pose.position.x - path.poses[i].pose.position.x) *
        (path.poses[k].pose.position.y - path.poses[j].pose.position.y) -
        (path.poses[j].pose.position.y - path.poses[i].pose.position.y) *
        (path.poses[k].pose.position.x - path.poses[j].pose.position.x);
    if (z < 0) {
      --count;
    } else if (z > 0) {
      ++count;
    }
  }

  ROS_WARN("path polygon is %s", count < 0 ? "closewise" : "anticlosewise");
  return count < 0;
}
}
