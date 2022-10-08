//
// Created by tony on 2022/10/8.
//

#include "explore.h"

namespace xju::explore {
	Explore::Explore()
			: move_base_client_(new MoveBaseActionClient("move_base", true)),
			  frontier_search_(new FrontierSearch),
			  tf_(new tf2_ros::Buffer()),
			  tfl_(new tf2_ros::TransformListener(*tf_)),
			  cancel_(false),
			  pause_(false),
			  resume_(false) {
		ROS_INFO("Explore initialized!");
		ros::NodeHandle n("/");
		vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	}

	Explore::~Explore() {
		pub_zero_vel();
	}

	void Explore::run() {
		ROS_INFO("Publish init static map.");
		frontier_search_->pub_static_map();
		ROS_INFO("Wait for move base online.");
		if (!move_base_client_->waitForServer(ros::Duration(WAIT_TIMEOUT))) {
			ROS_ERROR("move base not online, this shouldn't be!");
		}

		auto begin = ros::Time::now();

		// TODO: if map size enough, do not turn around.
		turn_around();

		auto ret = frontier_search_->get_frontier_point(robot_pose());

		while (ret) {
			if (cancel_) {
				ROS_INFO("Cancel explore!");
				return;
			}

			if (pause_) {
				ROS_INFO("Pause explore!");
				while (!resume_) {
					ros::Duration(1.0).sleep();
				}

				move_to_frontier(current_);
				ret = frontier_search_->get_frontier_point(robot_pose());
				continue;
			}

			ROS_INFO("Got a frontier [%.2f, %.2f, %.2f]", ret->x, ret->y, ret->yaw);
			current_ = ret.value();

			if (move_to_frontier(current_)) {
				ROS_INFO("move to frontier succeeded!");
			} else {
				ROS_INFO("go next.");
			}

			ret = frontier_search_->get_frontier_point(robot_pose());
		}

		ROS_WARN("[explore] Fail to get frontier point, goto origin pose!");
		move_to_frontier(Frontier{0, 0, 0});
		ROS_WARN("[explore] Stop explore, total time is %.2fs", (ros::Time::now() - begin).toSec());
	}

	auto Explore::move_to_frontier(Frontier const& fs) -> bool {
		using goal_state_type = actionlib::SimpleClientGoalState;

		move_base_msgs::MoveBaseGoal target;
		target.target_pose.header.stamp = ros::Time().now();
		target.target_pose.header.frame_id = MAP;
		target.target_pose.pose.position.x = fs.x;
		target.target_pose.pose.position.y = fs.y;
		target.target_pose.pose.orientation.z = std::sin(fs.yaw / 2);
		target.target_pose.pose.orientation.w = std::cos(fs.yaw / 2);

		ROS_INFO("Send move base goal.");
		auto ret = move_base_client_->sendGoalAndWait(target);

		if (ret == goal_state_type::SUCCEEDED) {
			ROS_INFO("move base action succeeded!");
			return true;
		} else {
			ROS_ERROR_STREAM("move base action failed (" << ret.getText() << ")!");
			return false;
		}
	}

	void Explore::turn_around() {
		ROS_INFO("start turning action");
		static double angular_z = ANGULAR_SPEED;
		angular_z *= -1;
		geometry_msgs::Twist vel;
		vel.linear.x = 0;
		vel.angular.z = angular_z;

		auto yaw = 0.0;
		while (yaw < 2 * M_PI) {
			if (cancel_) return;
			if (pause_) {
				pub_zero_vel();
				while (!resume_) {
					ros::Duration(1.0).sleep();
				}
			}

//			auto pose = robot_pose();
//			if (!pose) {
//				yaw += std::abs(angular_z / CONTROL_FREQ);
//				vel_pub_.publish(vel);
//				ros::Duration(1.0 / CONTROL_FREQ).sleep();
//				continue;
//			}
//
//			static geometry_msgs::Pose last = pose.value();
//			yaw += std::abs(tf2::getYaw(pose.value().orientation)
//			                - tf2::getYaw(last.orientation)); // not accurate
//			last = pose.value();

			yaw += std::abs(angular_z / CONTROL_FREQ);
			vel_pub_.publish(vel);
			ros::Duration(1.0 / CONTROL_FREQ).sleep();
		}

		pub_zero_vel();
		ROS_INFO("stop turning action");
	}

	void Explore::pub_zero_vel() {
		geometry_msgs::Twist vel;
		vel.linear.x = 0;
		vel.angular.z = 0;
		vel_pub_.publish(vel);
	}

	auto Explore::robot_pose() -> std::optional<geometry_msgs::Pose> {
		geometry_msgs::TransformStamped transformStamped;
		try {
			transformStamped = tf_->lookupTransform(MAP, "base_link", ros::Time(0));
		} catch (tf2::TransformException &ex) {
			ROS_WARN("tf error: %s", ex.what());
			return std::nullopt;
		}

		geometry_msgs::Pose result;
		result.position.x = transformStamped.transform.translation.x;
		result.position.y = transformStamped.transform.translation.y;
		result.orientation = transformStamped.transform.rotation;
		return std::make_optional(result);
	}
}
