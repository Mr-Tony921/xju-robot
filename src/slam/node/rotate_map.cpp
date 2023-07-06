//
// Created by tony on 2023/7/6.
//

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <boost/thread/mutex.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>

namespace xju::slam {
	static const double Degree2RadInv = M_PI / 180.0;

	class RotateMap {
	public:
		RotateMap();

		~RotateMap();

		void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

		void mapTimerCallback(const ros::TimerEvent& event);

		void tfTimerCallback(const ros::TimerEvent& event);

	private:
		ros::NodeHandle         node_;
		boost::mutex            map_mutex_;
		ros::Subscriber         carto_map_sub_;
		ros::Publisher          map_pub_;
		ros::Publisher          marker_pub_;
		ros::Timer              map_timer_;
		ros::Timer              tf_timer_;
		ros::ServiceClient      client_;
		ros::ServiceServer      display_object_ser_;
		nav_msgs::OccupancyGrid carto_map_;

		bool   map_updated_;
		double map_x_, map_y_, map_theta_;

		void map2Img(cv::Mat& mat);

		void getTheta(const cv::Mat& mat);

		void rotMap(nav_msgs::OccupancyGrid& map);
	};

	RotateMap::RotateMap() : map_x_(0.0), map_y_(0.0), map_theta_(0.0), map_updated_(false) {
		map_timer_ = node_.createTimer(ros::Duration(5.0), &RotateMap::mapTimerCallback, this);
		map_timer_.stop();
		tf_timer_      = node_.createTimer(ros::Duration(0.02), &RotateMap::tfTimerCallback, this);
		map_pub_       = node_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
		marker_pub_    = node_.advertise<visualization_msgs::Marker>("/current_longest", 1);
		carto_map_sub_ = node_.subscribe("carto_map", 1, &RotateMap::mapCallback, this);
	}

	RotateMap::~RotateMap() {
		map_timer_.stop();
		tf_timer_.stop();
	}

	void RotateMap::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
		boost::mutex::scoped_lock lock(map_mutex_);
		carto_map_   = *msg;
		map_updated_ = true;
		if (!map_timer_.hasStarted())
			map_timer_.start();
	}

	void RotateMap::mapTimerCallback(const ros::TimerEvent& event) {
		if (!map_updated_) return;

		cv::Mat carto_mat;

		nav_msgs::OccupancyGrid map = carto_map_;

		map2Img(carto_mat);
		getTheta(carto_mat);
		rotMap(map);
		map_pub_.publish(map);

		map_updated_ = false;
	}

	void RotateMap::tfTimerCallback(const ros::TimerEvent& event) {
		static tf::TransformBroadcaster br;
		tf::Transform                   transform;
		transform.setOrigin(tf::Vector3(map_x_, map_y_, 0.0));
		tf::Quaternion q;
		q.setRPY(0.0, 0.0, map_theta_);
			
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "carto_map"));
	}


	void RotateMap::map2Img(cv::Mat& mat) {
		boost::mutex::scoped_lock lock(map_mutex_);

		auto sizex = carto_map_.info.width;
		auto sizey = carto_map_.info.height;
		mat = cv::Mat(sizey, sizex, CV_8U);
		for (uint32_t r = 0; r < sizey; ++r) {
			for (uint32_t c = 0; c < sizex; ++c) {
				if (carto_map_.data.at(c + (sizey - r - 1) * sizex) < 75) {
					mat.at<uchar>(r, c) = 0;
				} else {
					mat.at<uchar>(r, c) = (uchar) carto_map_.data.at(c + (sizey - r - 1) * sizex);
				}
			}
		}
	}

	void RotateMap::getTheta(const cv::Mat& mat) {
		static double maxLength = 30.0; // 0.1 m/pixel
		cv::Mat mid;
		cv::Canny(mat, mid, 75, 99, 3);
		std::vector <cv::Vec4i> lines;
		cv::HoughLinesP(mid, lines, 1, CV_PI / 180, 30, maxLength * 0.9, 10);
		if (lines.empty()) return;

		auto max_ele = std::max_element(lines.begin(), lines.end(), [&](cv::Vec4i const& p1, cv::Vec4i const& p2) {
			return std::pow(p1[0] - p1[2], 2) + std::pow(p1[1] - p1[3], 2)
			       < std::pow(p2[0] - p2[2], 2) + std::pow(p2[1] - p2[3], 2);
		});
		auto x0 = static_cast<double>((*max_ele)[0]);
		auto x1 = static_cast<double>((*max_ele)[2]);
		auto y0 = static_cast<double>((*max_ele)[1]);
		auto y1 = static_cast<double>((*max_ele)[3]);
		auto length = hypot(x1 - x0, y1 - y0);
		auto theta = atan2(y1 - y0, x1 - x0);
		ROS_INFO("[rotate map] Scan map got %.2f m length and %.2f rad theta.", length, theta);
		if (length > 1.1 * maxLength || fabs(theta - map_theta_) > Degree2RadInv) {
			ROS_INFO("[rotate map] Update global theta.");
			maxLength = std::max(maxLength, length);
			map_theta_ = theta;
			if (fabs(map_theta_) < Degree2RadInv) {
				map_theta_ = 0.0;
			}

			visualization_msgs::Marker line_marker;
			line_marker.type = visualization_msgs::Marker::ARROW;
			line_marker.header.frame_id = "carto_map";
			line_marker.header.stamp = ros::Time::now();
			line_marker.ns = "current_longest";
			line_marker.action = visualization_msgs::Marker::ADD;
			line_marker.color.a = 100;
			line_marker.color.r = 0;
			line_marker.color.g = 0;
			line_marker.color.b = 1;
			line_marker.id = 0;
			line_marker.scale.x = 0.05;
			line_marker.scale.y = 0.08;
			geometry_msgs::Point p;
			p.x = carto_map_.info.origin.position.x + carto_map_.info.resolution * x0;
			p.y = carto_map_.info.origin.position.y + carto_map_.info.resolution * (carto_map_.info.height - 1 - y0);
			p.z = 0;
			line_marker.points.emplace_back(p);
			p.x = carto_map_.info.origin.position.x + carto_map_.info.resolution * x1;
			p.y = carto_map_.info.origin.position.y + carto_map_.info.resolution * (carto_map_.info.height - 1 - y1);
			line_marker.points.emplace_back(p);
			marker_pub_.publish(line_marker);
		}
	}


	void RotateMap::rotMap(nav_msgs::OccupancyGrid& map) {
		if (fabs(map_theta_) < Degree2RadInv) {
			map = carto_map_;
			map.header.frame_id = "map";
			map_x_     = 0.0;
			map_y_     = 0.0;
			map_theta_ = 0.0;
			return;
		}

		boost::mutex::scoped_lock lock(map_mutex_);
        auto cos_p = cos(map_theta_);
        auto sin_p = sin(map_theta_);
        auto cos_n = cos_p;
        auto sin_n = -sin_p;
        auto resolution = carto_map_.info.resolution;
        auto width_cell_old = carto_map_.info.width;
        auto height_cell_old = carto_map_.info.height;
        auto width_cell = static_cast<unsigned int>(fabs(width_cell_old * cos_p) + fabs(height_cell_old * sin_p));
        auto height_cell = static_cast<unsigned int>(fabs(width_cell_old * sin_p) + fabs(height_cell_old * cos_p));
        auto width_old = static_cast<float>(width_cell_old) * resolution;
        auto height_old = static_cast<float>(height_cell_old) * resolution;
        auto width = static_cast<float>(width_cell) * resolution;
        auto height = static_cast<float>(height_cell) * resolution;
        auto center_x = carto_map_.info.origin.position.x + width_old / 2;
        auto center_y = carto_map_.info.origin.position.y + height_old / 2;
        map.header.stamp = ros::Time::now();
        map.header.frame_id = "map";
        map.info.width = width_cell;
        map.info.height = height_cell;
        map.info.origin.position.x = center_x - width / 2;
        map.info.origin.position.y = center_y - height / 2;
        map.data.resize(width_cell * height_cell);
        map_x_ = center_x + (0.0 - center_x) * cos_p - (0.0 - center_y) * sin_p;
		map_y_ = center_y + (0.0 - center_x) * sin_p + (0.0 - center_y) * cos_p;
        for (unsigned int i = 0; i < width_cell; ++i) {
            for (unsigned int j = 0; j < height_cell; ++j) {
                auto wx = map.info.origin.position.x + static_cast<float>(i) * resolution;
                auto wy = map.info.origin.position.y + static_cast<float>(j) * resolution;
                auto wx_old = center_x + (wx - center_x) * cos_n - (wy - center_y) * sin_n;
                auto wy_old = center_y + (wx - center_x) * sin_n + (wy - center_y) * cos_n;
                auto ii = floor((wx_old - carto_map_.info.origin.position.x) / resolution + 0.5);
                auto jj = floor((wy_old - carto_map_.info.origin.position.y) / resolution + 0.5);
                if (ii < 0 || ii >= carto_map_.info.width || jj < 0 || jj >= carto_map_.info.height) {
                    map.data.at(i + j * map.info.width) = -1;
                } else {
                    map.data.at(i + j * map.info.width) = carto_map_.data.at(static_cast<unsigned long>(ii + jj * carto_map_.info.width));
                }
            }
        }
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "map_rotation");
	xju::slam::RotateMap _rotate_map;
	ros::spin();
	return 0;
}