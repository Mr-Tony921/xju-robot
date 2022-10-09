//
// Created by tony on 2022/10/8.
//

#pragma once

#include <nav_msgs/GetMap.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <boost/optional.hpp>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>

//#define DEBUG_MODE 1

namespace xju::explore {
constexpr static const double FRONTIER_RESO = 1.0;
constexpr static const double CONTOUR_LINE_MAX_GRID = 12;  // unit 0.1m
constexpr static const double CONTOUR_LINE_MIN_GRID = 3;
constexpr static const int INFLATION_SIZE = 2;
constexpr static const int POINT_SEARCH_RANGE = 12;

struct Frontier {
    double x;
    double y;
    double yaw;
    double sort;  // use border distance
};

enum class NAV_MAP_COST : int8_t { OBS = 100, FREE = 0, UNKNOWN = -1 };
enum class ORI_MAP_COST : int8_t { OBS = 50, FREE = 0, UNKNOWN = -1 };
enum class MAT_MAP_COST : uint8_t { OBS = 50, FREE = 0, UNKNOWN = 100 };

class FrontierSearch {
   public:
    FrontierSearch();

    ~FrontierSearch() = default;

    auto get_frontier_point(std::optional<geometry_msgs::Pose> const& pose) -> std::optional<Frontier>;

    void pub_static_map();

   private:
    void set_frontier_point(Frontier const& point);

    void find_frontier_point();

    void check_current_frontier();

    void map_cb(nav_msgs::OccupancyGridConstPtr const& msg);

    auto map_to_img(nav_msgs::OccupancyGrid const& map) -> cv::Mat;

    auto frontier_valid(Frontier const& point, bool opt = false) -> bool;

    auto frontier_optimize(Frontier& point) -> bool;

    void frontier_cost(nav_msgs::OccupancyGrid const& map, Frontier& point) const {
        if (!current_pose_) {
            ROS_ERROR("[explore] invalid current pose, go to border first.");
            // No.1 find points most near map boundary
            point.sort = -(std::abs(map.info.origin.position.x + map.info.width / 2 - point.x) +
                           std::abs(map.info.origin.position.y + map.info.height / 2 - point.y));
            return;
        }

        // No.2 find points most near current pose
        auto robot_pose = current_pose_.value();
        auto robot_yaw = tf2::getYaw(robot_pose.orientation);
        auto point_to_robot = std::atan2(point.y - robot_pose.position.y, point.x - robot_pose.position.x);
        point.sort = hypot(robot_pose.position.x - point.x, robot_pose.position.y - point.y) +
                     std::abs(robot_yaw - point_to_robot);
    }

    void frontier_preprocess(cv::Point2f const& cv_point, bool check = false);

    inline auto map_valid(nav_msgs::OccupancyGrid const& map, uint32_t i, uint32_t j) const -> bool {
        return (i < map.info.width) && (j < map.info.height);
    }

    inline auto map_index(nav_msgs::OccupancyGrid const& map, uint32_t i, uint32_t j) const -> uint32_t {
        return i + j * map.info.width;
    }

    inline auto wxgx(nav_msgs::OccupancyGrid const& map, double x) const -> uint32_t {
        return static_cast<uint32_t>((x - map.info.origin.position.x) / map.info.resolution);
    };

    inline auto wygy(nav_msgs::OccupancyGrid const& map, double y) const -> uint32_t {
        return static_cast<uint32_t>((y - map.info.origin.position.y) / map.info.resolution);
    };

    inline auto gxwx(nav_msgs::OccupancyGrid const& map, uint32_t i) const -> double {
        return static_cast<double>(map.info.origin.position.x + i * map.info.resolution);
    };

    inline auto gywy(nav_msgs::OccupancyGrid const& map, uint32_t j) const -> double {
        return static_cast<double>(map.info.origin.position.y + j * map.info.resolution);
    };

    template <typename E>
    inline constexpr auto to_under_type(E enumerator) noexcept -> std::underlying_type_t<E> {
        return static_cast<std::underlying_type_t<E>>(enumerator);
    }

   private:
    std::vector<Frontier> open_list_;
    std::vector<Frontier> close_list_;
    boost::mutex list_mutex_;
    boost::mutex map_mutex_;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid static_map_;
    std::atomic_bool map_update_;

    ros::NodeHandle node_;
    ros::Subscriber map_sub_;
    ros::Publisher map_pub_;

    std::optional<geometry_msgs::Pose> current_pose_;
};
}  // namespace xju::explore
