//
// Created by tony on 2022/10/25.
//

#include "FreeSpaceCalculator.h"

using namespace xju::planning;

auto FreeSpaceCalculator::find_bound_of_free_space(std::vector<CellIndex> const& free_space,
                                                   cv::Mat const& neural_mat)
-> std::tuple<int32_t, int32_t, int32_t, int32_t> {
  if (free_space.empty()) {
    return std::make_tuple(0, 0, neural_mat.cols, neural_mat.rows);
  }

  auto min_max_x = std::minmax_element(free_space.begin(),
                                       free_space.end(),
                                       [&](CellIndex const& cell_a,
                                           CellIndex const& cell_b) { return cell_a.col < cell_b.col; });
  auto min_max_y = std::minmax_element(free_space.begin(),
                                       free_space.end(),
                                       [&](CellIndex const& cell_a,
                                           CellIndex const& cell_b) { return cell_a.row < cell_b.row; });
  return std::make_tuple(min_max_x.first->col,
                         min_max_y.first->row,
                         min_max_x.second->col - min_max_x.first->col + 1,
                         min_max_y.second->row - min_max_y.first->row + 1);
}

auto FreeSpaceCalculator::get_free_cell_in_polygon(cv::Mat const& src,
                                                   cv::Mat& des,
                                                   std::vector<CellIndex>& free_space,
                                                   nav_msgs::Path const& planning_polygon,
                                                   std::shared_ptr<costmap_2d::Costmap2D> const& rotated_costmap,
                                                   bool border_contained) -> int32_t {
  if (!des.empty() || des.data != nullptr) {
    des.release();
  }

  des = cv::Mat(src.rows / model_size_cell_ + (src.rows % model_size_cell_ == 0 ? 0 : 1),
                src.cols / model_size_cell_ + (src.rows % model_size_cell_ == 0 ? 0 : 1),
                src.type());
  std::vector<CellIndex> polygon;
  int a, b;
  bool duplicated;
  ROS_INFO("after remove duplicated points, polygon corners(in rotated_costmap) are:");
  for (auto const& pose: planning_polygon.poses) {
    duplicated = false;
    rotated_costmap->worldToMapEnforceBounds(pose.pose.position.x, pose.pose.position.y, a, b);
    for (auto const& point: polygon) {
      if (point.row == (src.rows - b - 1) / model_size_cell_ && point.col == a / model_size_cell_) {
        duplicated = true;
      }
    }

    if (!duplicated) {
      CellIndex tmp;
      tmp.row = (src.rows - b - 1) / model_size_cell_;
      tmp.col = a / model_size_cell_;
      ROS_INFO("[x, y] = [%d, %d]", a, b);
      polygon.emplace_back(tmp);
    }
  }

  ROS_INFO("After remove duplicated points, polygon corners(in cells) are:");
  for (auto const& point: polygon) {
    ROS_INFO("[row, col] = [%d, %d]  ", point.row, point.col);
  }

  if (polygon.size() < 3) {
    ROS_ERROR("Got a polygon zone with less than 3 points. Give up coverage path planning.");
  }

  free_space.clear();
  cells_poses_.clear();
  ROS_INFO("CoveragePathPlanner: free_space_ in function get_free_cell_in_polygon");
  int r = 0, c = 0;
  for (r = 0; r < des.rows; r++) {
    std::vector<SimpleCellIndex> row_vec;
    for (c = 0; c < des.cols; c++) {
      CellIndex ci;
      ci.row = r;
      ci.col = c;
      row_vec.emplace_back(SimpleCellIndex{-1, -1});
      auto in_polygon_state = in_polygon_cell(ci, polygon);
      if (in_polygon_state == InPolygonState::out_of_polygon) {
        des.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;
        continue;
      }

      if (in_polygon_state == InPolygonState::in_border && border_contained) {
        auto in_polygon_state_map = in_polygon_map(ci, planning_polygon, rotated_costmap);
        if (in_polygon_state_map != InPolygonState::in_polygon) {
          des.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;
          continue;
        }
      }

      std::vector<SimpleCellIndex> temp_col_vec;
      for (auto i = 0; i < model_size_cell_; ++i) {
        for (auto j = 0; j < model_size_cell_; ++j) {
          if (r * model_size_cell_ + i >= 0
              && r * model_size_cell_ + i < src.rows
              && c * model_size_cell_ + j >= 0
              && c * model_size_cell_ + j < src.cols
              && src.at<uchar>(r * model_size_cell_ + i, c * model_size_cell_ + j) <= cell_free_thres_) {
            temp_col_vec.emplace_back(SimpleCellIndex{i, j});
          }
        }
      }

      if (temp_col_vec.empty()) {
        des.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;
        continue;
      }

      free_space.emplace_back(ci);
      des.at<uchar>(r, c) = costmap_2d::FREE_SPACE;
      //如果当前cell是边沿cell，默认走中间
      if (in_polygon_state == InPolygonState::in_border) {
        row_vec.back().row = model_size_cell_ / 2;
        row_vec.back().col = model_size_cell_ / 2;
        auto temp_row = r * model_size_cell_ + row_vec.back().row;
        auto temp_col = c * model_size_cell_ + row_vec.back().col;
        ROS_INFO("type4 cell: [%d, %d], mat: [%d, %d], costmap: [%d, %d], cost = %d, row range = [%d, %d]",
                 r, c, temp_row, temp_col, temp_col, src.rows - temp_row - 1,
                 static_cast<int32_t>(src.at<uchar>(temp_row, temp_col)),
                 src.rows - r * model_size_cell_ - 1,
                 src.rows - (r * model_size_cell_ + model_size_cell_ - 1) - 1);
        continue;
      }

      auto find_res = std::find_if(temp_col_vec.begin(),
                                   temp_col_vec.end(),
                                   [&](SimpleCellIndex const& pose1) {
                                     return src.at<uchar>(r * model_size_cell_ + pose1.row,
                                                          c * model_size_cell_ + pose1.col) > border_cost_;
                                   });
      //如果当前cell没有cost>0的点，则默认走中间行
      if (find_res == temp_col_vec.end()) {
        row_vec.back().row = model_size_cell_ / 2;
        row_vec.back().col = model_size_cell_ / 2;
        auto temp_row = r * model_size_cell_ + row_vec.back().row;
        auto temp_col = c * model_size_cell_ + row_vec.back().col;
        ROS_INFO("type1 cell: [%d, %d], mat: [%d, %d], costmap: [%d, %d], cost = %d, row range = [%d, %d]",
                 r, c, temp_row, temp_col, temp_col, src.rows - temp_row - 1,
                 static_cast<int32_t>(src.at<uchar>(temp_row, temp_col)),
                 src.rows - r * model_size_cell_ - 1,
                 src.rows - (r * model_size_cell_ + model_size_cell_ - 1) - 1);
        continue;
      }

      find_res = std::find_if(temp_col_vec.begin(),
                              temp_col_vec.end(),
                              [&](SimpleCellIndex const& pose1) {
                                return pose1.row == model_size_cell_ / 2;
                              });
      //如果当前cell中间行没有可通行点，则找可通行点中cost最大的点（使路径更接近柱子）
      if (find_res == temp_col_vec.end()) {
        auto max_elem = std::max_element(temp_col_vec.begin(),
                                         temp_col_vec.end(),
                                         [&](SimpleCellIndex const& pose1,
                                             SimpleCellIndex const& pose2) {
                                           return src.at<uchar>(r * model_size_cell_ + pose1.row,
                                                                c * model_size_cell_ + pose1.col)
                                                  < src.at<uchar>(r * model_size_cell_ + pose2.row,
                                                                  c * model_size_cell_ + pose2.col);
                                         });
        row_vec.back().row = max_elem->row;
        row_vec.back().col = max_elem->col;
        auto temp_row = r * model_size_cell_ + row_vec.back().row;
        auto temp_col = c * model_size_cell_ + row_vec.back().col;
        ROS_INFO("type2 cell: [%d, %d], mat: [%d, %d], costmap: [%d, %d], cost = %d, row range = [%d, %d]",
                 r, c, temp_row, temp_col, temp_col, src.rows - temp_row - 1,
                 static_cast<int32_t>(src.at<uchar>(temp_row, temp_col)),
                 src.rows - r * model_size_cell_ - 1,
                 src.rows - (r * model_size_cell_ + model_size_cell_ - 1) - 1);
        continue;
      }

      //否则，走中间行(及它两边两行中)cost最大的点，障碍吸附功能，尽可能清理柱子周围
      decltype(temp_col_vec) middle_row;
      while (find_res != temp_col_vec.end()) {
        middle_row.emplace_back(*find_res);
        find_res = std::find_if(find_res + 1,
                                temp_col_vec.end(),
                                [&](SimpleCellIndex const& pose1) {
                                  return std::abs(pose1.row - model_size_cell_ / 2) <= 1;
                                });
      }

      auto max_elem = std::max_element(middle_row.begin(),
                                       middle_row.end(),
                                       [&](SimpleCellIndex const& pose1,
                                           SimpleCellIndex const& pose2) {
                                         return src.at<uchar>(r * model_size_cell_ + pose1.row,
                                                              c * model_size_cell_ + pose1.col)
                                                < src.at<uchar>(r * model_size_cell_ + pose2.row,
                                                                c * model_size_cell_ + pose2.col)
                                                || (src.at<uchar>(r * model_size_cell_ + pose1.row,
                                                                  c * model_size_cell_ + pose1.col)
                                                    == src.at<uchar>(r * model_size_cell_ + pose2.row,
                                                                     c * model_size_cell_ + pose2.col)
                                                    && -std::abs(pose1.row - model_size_cell_ / 2)
                                                       - std::abs(pose1.col - model_size_cell_ / 2)
                                                       < -std::abs(pose2.row - model_size_cell_ / 2)
                                                         - std::abs(pose2.col - model_size_cell_ / 2))
                                                || (src.at<uchar>(r * model_size_cell_ + pose1.row,
                                                                  c * model_size_cell_ + pose1.col)
                                                    == src.at<uchar>(r * model_size_cell_ + pose2.row,
                                                                     c * model_size_cell_ + pose2.col)
                                                    && -std::abs(pose1.row - model_size_cell_ / 2)
                                                       - std::abs(pose1.col - model_size_cell_ / 2)
                                                       == -std::abs(pose2.row - model_size_cell_ / 2)
                                                          - std::abs(pose2.col - model_size_cell_ / 2)
                                                    && -std::abs(pose1.row - model_size_cell_ / 2)
                                                       < -std::abs(pose2.row - model_size_cell_ / 2));
                                       });
      row_vec.back().row = max_elem->row;
      row_vec.back().col = max_elem->col;
      auto temp_row = r * model_size_cell_ + row_vec.back().row;
      auto temp_col = c * model_size_cell_ + row_vec.back().col;
      ROS_INFO("type3 cell: [%d, %d], mat: [%d, %d], costmap: [%d, %d], cost = %d, row range = [%d, %d]",
               r, c, temp_row, temp_col, temp_col, src.rows - temp_row - 1,
               static_cast<int32_t>(src.at<uchar>(temp_row, temp_col)),
               src.rows - r * model_size_cell_ - 1,
               src.rows - (r * model_size_cell_ + model_size_cell_ - 1) - 1);
    }

    cells_poses_.emplace_back(row_vec);
  }

  ROS_INFO("Before optimize, free space size is %lu", free_space.size());
//	    imwrite(debug_path_ + "cellMat.jpg", des);
  return src.rows - des.rows * model_size_cell_;
}

auto FreeSpaceCalculator::in_polygon_map(CellIndex const& cell,
                                         nav_msgs::Path const& planning_polygon,
                                         std::shared_ptr<costmap_2d::Costmap2D> const& rotated_costmap) const -> InPolygonState {
  auto mx_of_cell = static_cast<int32_t>(cell.col * model_size_cell_ + model_size_cell_ / 2);
  auto my_of_cell = static_cast<int32_t>(rotated_costmap->getSizeInCellsY()
                                         - (cell.row * model_size_cell_ + model_size_cell_ / 2)
                                         - 1);
  std::vector<CellIndex> planning_polygon_map;
  int32_t mx, my;
  for (auto const& pose : planning_polygon.poses) {
    rotated_costmap->worldToMapEnforceBounds(pose.pose.position.x, pose.pose.position.y, mx, my);
    planning_polygon_map.emplace_back(CellIndex{my, mx, 0});
  }

  //构建一个跟map分辨率等大的cell
  CellIndex temp_cell{my_of_cell, mx_of_cell, 0};
  return in_polygon_cell(temp_cell, planning_polygon_map);
}

auto FreeSpaceCalculator::in_line_cell(CellIndex const& cell, int x0, int y0, int x1, int y1) -> bool {
  using namespace std;

  auto px = cell.row;
  auto py = cell.col;

  if (px > max(x0, x1) || px < min(x0, x1) || py > max(y0, y1) || py < min(y0, y1))
    return false;

  auto side = (py - y0) * (x1 - x0) - (px - x0) * (y1 - y0);

  return side == 0;
}

auto FreeSpaceCalculator::in_polygon_cell(CellIndex const& cell,
                                          std::vector<CellIndex> const& polygon) -> InPolygonState {
  using namespace std;

  auto size = polygon.size();
  if (size< 3) return InPolygonState::out_of_polygon;

  auto p_x = cell.row;
  auto p_y = cell.col;
  auto counter = 0;
  double xinters;
  int p1_x, p1_y, p2_x, p2_y;
  p1_x = polygon.back().row;
  p1_y = polygon.back().col;

  for (size_t i = 0; i < size; i++) {
    p2_x = polygon[i].row;
    p2_y = polygon[i].col;

    if (in_line_cell(cell, p1_x, p1_y, p2_x, p2_y)) {
      return InPolygonState::in_border;
    }

    if (p1_y == p2_y) {
      p1_x = p2_x;
      continue;
    }

    if (p_y >= min(p1_y, p2_y) && p_y < max(p1_y, p2_y) && p_x <= max(p1_x, p2_x)) {
      xinters = double((p_y - p1_y) * (p2_x - p1_x)) / (p2_y - p1_y) + p1_x;
      if (p1_x == p2_x || p_x <= xinters) {
        counter++;
      }
    }

    p1_x = p2_x;
    p1_y = p2_y;
  }

  if (counter % 2 != 0) return InPolygonState::in_polygon;
  return InPolygonState::out_of_polygon;
}

