//
// Created by tony on 2022/11/4.
//

#include <ros/ros.h>

#include <csignal>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace xju::pnc {
class WallFollow {
  enum class WallFollowDirection {RIGHT = 0, LEFT = 1};
  static const constexpr double FRONT_LENGTH = 0.3;
  static const constexpr double HALF_WIDTH = 0.2;
  static const constexpr double MARGIN = 0.05;
  static const constexpr double LASER_BASE = 0.29;
  static const constexpr double SIDE_TARGET = 0.1;

public:
  WallFollow() : direction_(WallFollowDirection::RIGHT),
                 front_(std::numeric_limits<double>::max()),
                 side_(std::numeric_limits<double>::max()),
                 meet_(false),
                 bye_(false) {
    ros::NodeHandle nh("/");
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("wf_marker", 1);
    scan_sub_ = nh.subscribe("scan", 1, &xju::pnc::WallFollow::scan_callback, this, ros::TransportHints().tcpNoDelay());
    vel_timer_ = nh.createTimer(ros::Duration(0.1),
                                [&](const ros::TimerEvent& e) {
                                  cur_vel_.linear.x = get_lon_vel(); // 纵向速度
                                  cur_vel_.angular.z = get_lat_vel(); // 航向速度
                                  vel_pub_.publish(cur_vel_);
                                  ROS_INFO("meet %d bye %d front %.3f side %.3f v %.3f w %.3f",
                                           meet_, bye_, front_, side_, cur_vel_.linear.x, cur_vel_.angular.z);
                                },
                                false, false);
  }

  ~WallFollow() {
    stop();
  }

  void start() {
    publish_zero_velocity();
    vel_timer_.start();
  }

  void stop() {
    vel_timer_.stop();
    publish_zero_velocity();
  }

private:
  void publish_zero_velocity() {
    cur_vel_.linear.x = 0.;
    cur_vel_.angular.z = 0.;
    vel_pub_.publish(cur_vel_);
  }

  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    static std::vector<std::pair<double, double>> cos_sin;
    static bool got_cos_sin = false;
    if (!got_cos_sin) {
      got_cos_sin = true;
      cos_sin.resize(msg->ranges.size());
      for (auto i = 0; i < msg->ranges.size(); ++i) {
        cos_sin[i].first = std::cos(msg->angle_min + i * msg->angle_increment);
        cos_sin[i].second = std::sin(msg->angle_min + i * msg->angle_increment);
      }
    }

    visualization_msgs::Marker wf_points;
    wf_points.header.stamp = ros::Time::now();
    wf_points.header.frame_id = "base_link";
    wf_points.type = visualization_msgs::Marker::POINTS;
    wf_points.action = visualization_msgs::Marker::ADD;
    wf_points.scale.x = 0.2;
    wf_points.scale.y = 0.2;
    front_ = std::numeric_limits<double>::max();
    side_ = std::numeric_limits<double>::max();
    double x, y;
    geometry_msgs::Point pf, ps;
    for (auto i = 0; i < msg->ranges.size(); ++i) {
      x = msg->ranges[i] * cos_sin[i].first + LASER_BASE;
      y = msg->ranges[i] * cos_sin[i].second;
      if (x > FRONT_LENGTH && std::abs(y) < HALF_WIDTH + MARGIN) {
        // front area
        if (x - FRONT_LENGTH < front_) {
          front_ = x - FRONT_LENGTH;
          pf.x = x;
          pf.y = y;
        }
        continue;
      }
      if (direction_ == WallFollowDirection::RIGHT && x < FRONT_LENGTH && y < -HALF_WIDTH) {
        // right side area
        if (-y - HALF_WIDTH < side_) {
          side_ = -y - HALF_WIDTH;
          ps.x = x;
          ps.y = y;
        }
        continue;
      }
      if (direction_ == WallFollowDirection::LEFT && x < FRONT_LENGTH && y > HALF_WIDTH) {
        // left side area
        if (y - HALF_WIDTH < side_) {
          side_ = y - HALF_WIDTH;
          ps.x = x;
          ps.y = y;
        }
      }
    }
    std_msgs::ColorRGBA color;
    if (front_ < 10.0) {
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 0.8;
      wf_points.points.emplace_back(pf);
      wf_points.colors.emplace_back(color);
    }
    if (side_ < 10.0) {
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.8;
      wf_points.points.emplace_back(ps);
      wf_points.colors.emplace_back(color);
    }
    marker_pub_.publish(wf_points);
  }

  double get_lon_vel() {
    const double v_max = 0.8;
    const double l_max = 1.0;
    meet_ = front_ < SIDE_TARGET + 0.061 + 0.05;
    auto coff = std::min(1.0, front_ / l_max);
    if (meet_ || bye_) coff *= 0.1;
    return coff * v_max;
  }

  double get_lat_vel() {
    const double w_max = 0.5;
    auto delta = side_ - SIDE_TARGET;
    bye_ = delta > 2 * SIDE_TARGET;
    if (meet_) return w_max * (direction_ == WallFollowDirection::RIGHT ? 1.0 : -1.0);
    if (std::abs(delta) > 1.0) delta /= std::abs(delta);
    auto coff = delta * (direction_ == WallFollowDirection::RIGHT ? -1.0 : 1.0);
    return coff * w_max;
  }

private:
  geometry_msgs::Twist cur_vel_;

  ros::Publisher vel_pub_, marker_pub_;
  ros::Subscriber scan_sub_;
  ros::Timer vel_timer_;

  WallFollowDirection direction_;
  double front_, side_;
  bool meet_, bye_;
};
}

std::shared_ptr<xju::pnc::WallFollow> wf_ptr;

void sigintHandler(int sig) {
  if (wf_ptr) {
    wf_ptr->stop();
    wf_ptr.reset();
  }
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "xju_wall_follow");

  wf_ptr = std::make_shared<xju::pnc::WallFollow>();
  signal(SIGINT, sigintHandler);
  wf_ptr->start();
  ros::spin();

  return 0;
}
