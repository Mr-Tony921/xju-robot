//
// Created by tony on 2023/6/8.
//

#include "video_recorder.h"

namespace xju::rec {
VideoRecorder::VideoRecorder() : cv_image_ptr_(nullptr) {
    ros::NodeHandle pnh("~");
    pnh.param<int>("video_duration", duration_, 300);
    pnh.param<double>("video_fps", fps_, 10.);

    image_sub_ = pnh.subscribe("/camera_front/image", 1, &VideoRecorder::image_cb, this);
    odom_sub_ = pnh.subscribe("/odom", 1, &VideoRecorder::odom_cb, this, ros::TransportHints().tcpNoDelay());

    open_writer();

    record_ = true;
}

void VideoRecorder::start_recorder() {
    ros::Rate r(ros::Duration(1. / fps_));
    while (record_) {
        ros::spinOnce();
        if (!cv_image_ptr_) {
            r.sleep();
            continue;
        }

        static double last = ros::Time::now().toSec();
        double now = ros::Time::now().toSec();
        if (now - last > static_cast<double>(duration_)) {
            close_writer();
            last = now;
            open_writer();
        }

        text_image(cv_image_ptr_, odom_);
        video_writer_.write(cv_image_ptr_->image);
        r.sleep();
    }
}

void VideoRecorder::finish_recorder() {
    record_ = false;
    close_writer();
}

void VideoRecorder::image_cb(const sensor_msgs::Image::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        cv_image_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void VideoRecorder::odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_ = *msg;
}

void VideoRecorder::text_image(cv_bridge::CvImagePtr image_ptr, const nav_msgs::Odometry& odom) {
    auto time = [&]() {
        return std::string("time: ") + now();
    };

    auto speed = [&]() {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << odom_.twist.twist.linear.x;
        return std::string("speed: ") + oss.str() + std::string("m/s");
    };

    std::lock_guard<std::mutex> lock(mutex_);
    cv::Scalar text_color(255, 255, 255);

    cv::putText(image_ptr->image, time(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, text_color, 2, cv::LINE_AA);
    cv::putText(image_ptr->image, speed(), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, text_color, 2, cv::LINE_AA);
}

void VideoRecorder::open_writer() {
    std::string filename = "/home/tony/course_ws/rec/ongoing.mp4";
    int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    cv::Size frame_size(1280, 720);
    video_writer_.open(filename, codec, fps_, frame_size);
    video_start_time_ = now();
}

void VideoRecorder::close_writer() {
    std::string oldname = "/home/tony/course_ws/rec/ongoing.mp4";
    if (video_writer_.isOpened()) {
        video_writer_.release();
    }

    ROS_INFO("Roll back video file");
    auto newname = std::string("/home/tony/course_ws/rec/video_") + video_start_time_ + std::string(".mp4");
    ROS_ERROR_COND(std::rename(oldname.c_str(), newname.c_str()) != 0, "Rename failed!");
}

auto VideoRecorder::now() -> std::string {
    std::time_t now = std::time(nullptr);
    std::tm *localTime = std::localtime(&now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localTime);
    return std::string(buffer);
}
} // namespace xju::rec
