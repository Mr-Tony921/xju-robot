//
// Created by tony on 2023/6/8.
//

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <ctime>
#include <mutex>
#include <vector>
#include <iostream>
#include <sstream>
#include <iomanip>

namespace xju::rec {
constexpr static const char* NODE_NAME = "video_recorder";

class VideoRecorder {
   public:
    VideoRecorder();

    ~VideoRecorder() = default;

    void start_recorder();

    void finish_recorder();

   private:
    void image_cb(const sensor_msgs::Image::ConstPtr& msg);

    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);

    void text_image(cv_bridge::CvImagePtr image_ptr, const nav_msgs::Odometry& odom);

    void open_writer();

    void close_writer();

    auto now() -> std::string;

   private:
    cv_bridge::CvImagePtr cv_image_ptr_;
    nav_msgs::Odometry odom_;

    cv::VideoWriter video_writer_;

    int duration_;
    double fps_;
    std::string video_start_time_;

    ros::Subscriber image_sub_, odom_sub_;
    ros::Timer timer_;

    std::atomic_bool record_;
    std::mutex mutex_;

};
}  // namespace xju::rec
