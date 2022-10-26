//
// Created by tony on 2022/10/25.
//

#pragma once

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>
#include <cmath>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "BezierTrajectoryGeneratorWaypoint.h"
#include "FreeSpaceCalculator.h"

namespace xju::planning {
//    #define DEBUG_MODE
//#define REVERSE_COARSE_PATH

static constexpr int LOOP_MAX = 9000;
static constexpr long CHECK_MAX = 20;
static constexpr double ZERO = 1E-5;
static constexpr double AMPLIFY = 2.0;
static constexpr double CHECK_DIST = 2;
static constexpr double FIL_TOLER = 0.5;
static constexpr float C_0 = 50;
static constexpr float OUT_SCORE = -250;
static constexpr float WEIGHT_TINY = -1;
static constexpr double PATH_STEP = 0.05;


class CoveragePathPlanner {
  enum class CoarsePathSeparate : int {
    delete_point = 1, break_point = 2
  }; //1:直线中点，需要删除，2:间断点，该点前后的点分属于不同组

  enum class CleanAreaType : uint8_t {
    zigzag = 0, backshaped = 1
  }; //0:普通覆盖，1：车位覆盖，2：回字覆盖
public:
  CoveragePathPlanner(nav_msgs::OccupancyGrid const& grid,
                      nav_msgs::Path const& polygon,
                      uint8_t clean_area_type = 0);

  ~CoveragePathPlanner() = default;

  auto get_path_interface() -> std::vector<nav_msgs::Path>;

private:
  void on_init();

  auto initialize_mats() -> bool;

  void initialize_neural_mat(cv::Mat const& src, cv::Mat& des) const;

  auto get_zigzag_indexed_path() -> std::optional<std::vector<CellIndex>>;

  auto get_backshaped_indexed_path() -> std::optional<std::vector<CellIndex>>;

  /*用bezier曲线计算fine_path_*/
  auto get_bezier_optimized_path(nav_msgs::Path const& path) const -> nav_msgs::Path;

  void adjust_border(nav_msgs::Path const& cover, nav_msgs::Path& border);

  auto save_border(nav_msgs::Path& border) -> bool;

  auto get_free_cell_in_polygon(cv::Mat const& src, cv::Mat& des, std::vector<CellIndex>& free_space)
  -> std::vector<std::vector<SimpleCellIndex>>;

  auto get_start_point(nav_msgs::Path const& polygon)
  -> std::optional<std::tuple<double, double, uint32_t, uint32_t, CellIndex>>;

  auto calculate_polygon_inclination(nav_msgs::Path const& polygon, double tolerance) -> double;

  auto polygon_cutter(nav_msgs::Path const& polygon) const -> nav_msgs::Path;

  void rotate_map_polygon(double angle);

  auto grid2costmap(nav_msgs::OccupancyGrid const& grid) const -> std::shared_ptr<costmap_2d::Costmap2D>;

  auto costmap2cv_mat(std::shared_ptr<costmap_2d::Costmap2D> const& costmap,
                      int32_t x,
                      int32_t y,
                      int32_t width,
                      int32_t height) -> cv::Mat;

  auto costmap2cv_mat(std::shared_ptr<costmap_2d::Costmap2D> const& costmap) -> cv::Mat {
    int32_t x = 0;
    int32_t y = 0;
    auto width = static_cast<int32_t>(costmap->getSizeInCellsX());
    auto height = static_cast<int32_t>(costmap->getSizeInCellsY());
    return costmap2cv_mat(costmap, x, y, width, height);
  };

  inline auto dist2(geometry_msgs::PoseStamped const& p1,
                    geometry_msgs::PoseStamped const& p2) const -> double {
    return std::pow((p1.pose.position.x - p2.pose.position.x), 2)
           + std::pow((p1.pose.position.y - p2.pose.position.y), 2);
  };

  /*
   * 根据起点的位置改变coarse_path_路径计算方向，weight_tiny_为正值时优先向下计算，反之，优先向上计算
   * param:
   * first_y: rotated_costmap地图坐标系下y坐标
   * */
  void change_weight_tiny(uint32_t first_y);

  /*对路径滤波，输出路径确保相邻路径点位于不同栅格*/
  auto border_filter(nav_msgs::Path const& path) -> nav_msgs::Path;

  /*对fine_path_中的border进行滤波，如果三个点，两两之间的*/
  auto border_direction_filter(nav_msgs::Path& path) -> nav_msgs::Path;

  void read_param();

  auto is_clockwise(nav_msgs::Path const& path) -> bool;

private:

  std::vector<CellIndex> free_space_;

  std::vector<nav_msgs::Path> coarse_path_;
  std::vector<nav_msgs::Path> fine_path_;

  std::vector<std::vector<SimpleCellIndex>> cells_poses_; //每个cell实际可以通行的pose，对于远离障碍的cell，就是cell中心pose，对于接近障碍的cell，则是最接近中心的，小于障碍阈值的pose，完全处于障碍中的cell，则是(-1, -1)

  nav_msgs::Path planning_polygon_;
  nav_msgs::Path border_polygon_;
  nav_msgs::Path origin_border_;

  std::shared_ptr<costmap_2d::Costmap2D> origin_costmap_;
  std::shared_ptr<costmap_2d::Costmap2D> rotated_costmap_;

  cv::Point2d center_;
  cv::Point2d polygon_center_;

  cv::Mat rotated_mat_;
  cv::Mat scale_mat_;
  cv::Mat neural_mat_;

  bool initialized_;

  int model_size_cell_;
  int cell_free_thres_;

  uint8_t lethal_cost_;

  float weight_tiny_;

  double angle_;
  double resolution_;

  std::string debug_path_;

  CleanAreaType clean_area_type_;
};
}
