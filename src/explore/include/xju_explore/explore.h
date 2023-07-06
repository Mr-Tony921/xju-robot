//
// Created by tony on 2022/10/8.
//

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "frontier_search.h"

namespace xju::explore {
constexpr static const char* NODE_NAME = "xju_explore";
constexpr static const double ANGULAR_SPEED = 0.3;
constexpr static const double CONTROL_FREQ = 5.0;
constexpr static const double WAIT_TIMEOUT = 10.0;
constexpr static const double DIST_RESO = 2.0;
constexpr static const char* MAP = "carto_map";

class Explore {
   public:
    using MoveBaseActionClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

   public:
    Explore();

    ~Explore();

    void run();

    void pub_zero_vel();

    void cancel() {
        cancel_ = true;
        if (!move_base_client_->getState().isDone()) {
            ROS_INFO("Cancel current move base goal.");
            move_base_client_->cancelGoal();
        }

        pub_zero_vel();
    };

    void pause() {
        resume_ = false;
        pause_ = true;
        if (!move_base_client_->getState().isDone()) {
            ROS_INFO("Pause current move base goal.");
            move_base_client_->cancelGoal();
        }

        pub_zero_vel();
    }

    void resume() {
        resume_ = true;
        pause_ = false;
    }

   private:
    auto move_to_frontier(Frontier const& fs) -> bool;

    void turn_around();

    auto robot_pose() -> std::optional<geometry_msgs::Pose>;

   private:
    std::unique_ptr<MoveBaseActionClient> move_base_client_;
    std::unique_ptr<FrontierSearch> frontier_search_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::unique_ptr<tf2_ros::TransformListener> tfl_;

    std::atomic_bool cancel_;
    std::atomic_bool pause_;
    std::atomic_bool resume_;

    ros::Publisher vel_pub_;

    Frontier current_;
};
}  // namespace xju::explore
