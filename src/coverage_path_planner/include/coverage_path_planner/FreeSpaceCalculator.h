//
// Created by tony on 2022/10/25.
//

#pragma once

#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace xju::planning {

struct CellIndex {
int32_t row = 0;
int32_t col = 0;
double theta = 0.0;
};

struct SimpleCellIndex {
int32_t row = 0;
int32_t col = 0;
};

enum class InPolygonState : int32_t {
  out_of_polygon = -1,
  in_border = 1, //在边界
  in_polygon = 2  //在多边形内部，不包含边界
};

class FreeSpaceCalculator {
public:
  FreeSpaceCalculator(int32_t cell_free_thres,
                      int32_t model_size_cell,
                      uint8_t border_cost)
    : cell_free_thres_(cell_free_thres),
      model_size_cell_(model_size_cell),
      border_cost_(border_cost) {}

  static auto find_bound_of_free_space(std::vector<CellIndex> const& free_space,
                                       cv::Mat const& neural_mat)
  -> std::tuple<int32_t, int32_t, int32_t, int32_t>;

  /**
   *
   * @param src
   * @param des
   * @param free_space
   * @param planning_polygon
   * @param rotated_costmap
   * @param border_contained
   * @return 返回值为des和src在y方向长度的差（统一到src的像素单位，通常为0.1m）
   */
  auto get_free_cell_in_polygon(cv::Mat const& src,
                                cv::Mat& des,
                                std::vector<CellIndex>& free_space,
                                nav_msgs::Path const& planning_polygon,
                                std::shared_ptr<costmap_2d::Costmap2D> const& rotated_costmap,
                                bool border_contained) -> int32_t;

  inline auto get_cells_poses() -> std::vector<std::vector<SimpleCellIndex>> { return cells_poses_; }

  static auto in_line_cell(CellIndex const& cell, int x0, int y0, int x1, int y1) -> bool;

  /*
   * 在cell分辨率下看，一个cell是否在polygon中
   */
  static auto in_polygon_cell(CellIndex const& cell,
                              std::vector<CellIndex> const& polygon) -> InPolygonState;

private:

  /*
   * 在地图坐标系的分辨率下看，一个cell的中心点是否在planning_polygon中。
   */
  auto in_polygon_map(CellIndex const& cell,
                      nav_msgs::Path const& planning_polygon,
                      std::shared_ptr<costmap_2d::Costmap2D> const& rotated_costmap) const -> InPolygonState;

private:
  int32_t cell_free_thres_;
  int32_t model_size_cell_;
  uint8_t border_cost_;

  std::vector<std::vector<SimpleCellIndex>> cells_poses_;
};
}
