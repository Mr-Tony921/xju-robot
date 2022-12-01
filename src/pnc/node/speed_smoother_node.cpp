//
// Created by tony on 2022/11/17.
//

#include <ros/ros.h>

#include <csignal>
#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>

namespace xju::pnc {

template<class T> inline T sgn(T val) { return val > 0 ? 1.0 : -1.0; }

class SpeedSmoother {
  static const constexpr double INPUT_FRQ = 10.0;
  static const constexpr double OUTPUT_FRQ = 50.0;
  static const constexpr double EPSILON = 1e-5;
  static const constexpr double LIN_ACC = 2 / OUTPUT_FRQ;
  static const constexpr double AGU_ACC = 1 / OUTPUT_FRQ;
  static const constexpr int PHASE = static_cast<int>(OUTPUT_FRQ / INPUT_FRQ);
  static const constexpr int FIT_NUM = 3;

public:
  SpeedSmoother() {
    ros::NodeHandle nh("/");
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("output", 1);
    vel_sub_ = nh.subscribe("input", 1, &xju::pnc::SpeedSmoother::vel_callback, this, ros::TransportHints().tcpNoDelay());
    vel_timer_ = nh.createTimer(ros::Duration(1.0 / OUTPUT_FRQ),
                                [&](const ros::TimerEvent& e) {
                                  if (linear_.empty() || angular_.empty()) return;
                                  cur_vel_.linear.x += delta_linear_;
                                  cur_vel_.angular.z += delta_angular_;
                                  up_down_limit(cur_vel_.linear.x, 0, 0.8);
                                  up_down_limit(cur_vel_.angular.x, -0.5, 0.5);
                                  vel_pub_.publish(cur_vel_);
                                },
                                false, false);
  }

  ~SpeedSmoother() {
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

  void vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (std::abs(msg->linear.x) < EPSILON && std::abs(msg->angular.z) < EPSILON) {
      linear_.clear();
      angular_.clear();
      cur_vel_.linear.x = 0.0;
      cur_vel_.angular.y = 0.0;
    }

    linear_.emplace_back(msg->linear.x);
    angular_.emplace_back(msg->angular.z);
    if (linear_.size() > FIT_NUM) linear_.erase(linear_.begin());
    if (angular_.size() > FIT_NUM) angular_.erase(angular_.begin());
    auto l = least_square_predict(linear_, 2);
    auto a = least_square_predict(angular_, 2);
    auto target_linear = l * linear_.back() <= 0 ? linear_.back() : l;
    delta_linear_ = (target_linear - cur_vel_.linear.x) / PHASE;
    if (std::abs(delta_linear_) > LIN_ACC) delta_linear_ = sgn(delta_linear_) * LIN_ACC;
    auto target_angular = a * angular_.back() <= 0 ? angular_.back() : a;
    delta_angular_ = (target_angular - cur_vel_.angular.z) / PHASE;
    if (std::abs(delta_angular_) > AGU_ACC) delta_angular_ = sgn(delta_angular_) * AGU_ACC;
    counter_ = 0;
  }

  double least_square_predict(std::vector<double> const& val, int order) {
    auto size = val.size();
    if (order < 1 || order > size - 1) return 0.0;

    Eigen::VectorXd xvals(size), yvals(size);
    for (auto i = 0 ; i < size; ++i) {
      xvals(i) = static_cast<double>(i);
      yvals(i) = val[i];
    }

    Eigen::MatrixXd A(size, order + 1);
    for (auto i = 0; i < size; ++i) {
      A(i, 0) = 1.0;
    }

    for (auto j = 0; j < size; ++j) {
      for (auto i = 0; i < order; ++i) {
        A(j, i + 1) = A(j, i) * xvals(j);
      }
    }

    auto Q = A.householderQr();
    auto coff = Q.solve(yvals);
    double result = 0.0;
    for (auto i = 0; i < coff.size(); ++i) {
      result += coff[i] * std::pow(size, i);
    }

    return result;
  }

  double least_square_predict(std::vector<double> const& val) {
    if (val.empty()) return 0.0;
    if (val.size() < 2) return 2 * val[0];

    double t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0;
    for (auto i = 0; i < val.size(); ++i) {
      t1 += i * i;
      t2 += i;
      t3 += i * val[i];
      t4 += val[i];
    }
    double a = (t3 * val.size() - t2 * t4) / (t1 * val.size() - t2 * t2);
    double b = (t1 * t4 - t2 * t3) / (t1 * val.size() - t2 * t2);
    return a * val.size() + b;
  }

  void up_down_limit(double& val, double down, double up) {
    if (val < down) val = down;
    if (val > up) val = up;
  }

private:
  ros::Publisher vel_pub_;
  ros::Subscriber vel_sub_;
  ros::Timer vel_timer_;

  geometry_msgs::Twist cur_vel_;

  std::vector<double> linear_, angular_;
  double delta_linear_, delta_angular_;

  std::atomic_int counter_;
};
}

std::shared_ptr<xju::pnc::SpeedSmoother> ss_ptr;

void sigintHandler(int sig) {
  if (ss_ptr) {
    ss_ptr->stop();
    ss_ptr.reset();
  }
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "xju_speed_smoother");

  ss_ptr = std::make_shared<xju::pnc::SpeedSmoother>();
  signal(SIGINT, sigintHandler);
  ss_ptr->start();
  ros::spin();

  return 0;
}

