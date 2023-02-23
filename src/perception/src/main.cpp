//
// Created by tony on 2023/2/21.
//

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>

#include "utils.h"

ros::Publisher _cloud_pub_1;
ros::Publisher _cloud_pub_2;
ros::Publisher _cloud_pub_3;
ros::Publisher _marker_pub_1;
ros::Publisher _marker_pub_2;
ros::Publisher _marker_pub_3;

static std::unique_ptr<xju::perception::RangeFilter> _range_filter;
static std::unique_ptr<xju::perception::GroundFilter> _ground_filter;
static std::unique_ptr<xju::perception::Segment> _segment;
static std::unique_ptr<xju::perception::Tracking> _tracking;

void points_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                     const nav_msgs::OdometryConstPtr& odom_msg) {
  auto start = ros::Time::now();

  /// 1. transform to general structure
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  /// 2. remove points by range
  pcl::PointCloud<pcl::PointXYZI>::Ptr range_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  *range_filtered_cloud = _range_filter->removePointsByRange(*cloud, 0.0, 6.0);

  /// 3. remove points in vehicle
  _range_filter->removePointsInCar(range_filtered_cloud);

  /// 4. ground filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZI>());
  _ground_filter->process(range_filtered_cloud, ground, non_ground);

  sensor_msgs::PointCloud2 pub_msg;
  pcl::toROSMsg(*ground, pub_msg);
  pub_msg.header = cloud_msg->header;
  _cloud_pub_1.publish(pub_msg);

  pcl::toROSMsg(*non_ground, pub_msg);
  pub_msg.header = cloud_msg->header;
  _cloud_pub_2.publish(pub_msg);

  /// 5. segmentation
  std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters;
  _segment->dbScan(non_ground, clusters);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  // different clusters with different intensity
  float step_i = 255.0f / clusters.size();
  pcl::PointXYZI point;
  for (size_t cluster_idx = 0u; cluster_idx < clusters.size(); ++cluster_idx) {
    if (clusters[cluster_idx].points.empty()) continue;
    for (size_t idx = 0u; idx < clusters[cluster_idx].points.size(); ++idx) {
      point.x = clusters[cluster_idx].points[idx].x;
      point.y = clusters[cluster_idx].points[idx].y;
      point.z = clusters[cluster_idx].points[idx].z;
      point.intensity = cluster_idx * step_i;
      cluster_cloud->points.emplace_back(point);
    }
  }

  pcl::toROSMsg(*cluster_cloud, pub_msg);
  pub_msg.header = cloud_msg->header;
  _cloud_pub_3.publish(pub_msg);

  /// 6. tracking
  _tracking->generateTrackingData(clusters, cloud_msg->header.stamp.toSec(), cloud_msg->header.seq);
  auto tracking_result = _tracking->process();

  if (!tracking_result.objects.empty()) {
    visualization_msgs::MarkerArray speed_marker_array;
    for (auto i = 0; i < tracking_result.objects.size(); ++i) {
      double velocity = std::hypot(tracking_result.objects[i].velocity.linear.x, tracking_result.objects[i].velocity.linear.y);
      if (velocity < 0.01) continue;

      visualization_msgs::Marker text_marker;
      text_marker.header.frame_id = "top_laser_link";
      text_marker.header.stamp = cloud_msg->header.stamp;
      text_marker.ns = "basic_shapes";
      text_marker.action = visualization_msgs::Marker::ADD;
      text_marker.pose.orientation.w = 1.0;
      text_marker.id = i;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

      text_marker.scale.z = 0.4;
      text_marker.color.b = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.a = 0.8;

      geometry_msgs::Pose pose;
      pose.position.x = tracking_result.objects[i].pose.position.x;
      pose.position.y = tracking_result.objects[i].pose.position.y;
      pose.position.z = 0;

      int id = tracking_result.objects[i].id;

      std::ostringstream str;
      str << "id: " << id << " velocity: "<< velocity;
      text_marker.text = str.str();
      text_marker.pose = pose;

      visualization_msgs::Marker arrow_marker;
      arrow_marker.header.frame_id = "top_laser_link";
      arrow_marker.header.stamp = cloud_msg->header.stamp;
      arrow_marker.ns = "lines";
      arrow_marker.action = visualization_msgs::Marker::ADD;
      arrow_marker.pose.orientation.w = 1.0;
      arrow_marker.id = tracking_result.objects.size() + i;
      arrow_marker.type = visualization_msgs::Marker::ARROW;

      arrow_marker.scale.x = 0.1;
      arrow_marker.scale.y = 0.3;
      arrow_marker.scale.z = 0.0;
      arrow_marker.color.b = 0;
      arrow_marker.color.g = 0;
      arrow_marker.color.r = 1.0;
      arrow_marker.color.a = 0.8;

      geometry_msgs::Point p1;
      p1.x = tracking_result.objects[i].pose.position.x;
      p1.y = tracking_result.objects[i].pose.position.y;
      p1.z = 0;
      geometry_msgs::Point p2;
      double length = std::hypot(tracking_result.objects[i].velocity.linear.x, tracking_result.objects[i].velocity.linear.y);

      p2.x = p1.x + tracking_result.objects[i].velocity.linear.x * 2.0 / length;
      p2.y = p1.y + tracking_result.objects[i].velocity.linear.y * 2.0 / length;
      p2.z = 0;
      arrow_marker.points.emplace_back(p1);
      arrow_marker.points.emplace_back(p2);

      speed_marker_array.markers.emplace_back(text_marker);
      speed_marker_array.markers.emplace_back(arrow_marker);
    }
    _marker_pub_1.publish(speed_marker_array);

    xju::perception::BoundingBox bounding_box(tracking_result, cloud_msg->header.stamp);
    visualization_msgs::MarkerArray prism_box = bounding_box.prisms_.toRviz();
    _marker_pub_2.publish(prism_box);

    visualization_msgs::MarkerArray cuboid_box = bounding_box.cuboids_.toRviz();
    _marker_pub_3.publish(cuboid_box);
  }

  auto duration = (ros::Time::now() - start).toSec();
//  ROS_INFO("Spend %.3f seconds", duration);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "xju_perception");
  ros::NodeHandle nh;

  _range_filter.reset(new xju::perception::RangeFilter());
  _ground_filter.reset(new xju::perception::GroundFilter());
  _segment.reset(new xju::perception::Segment());
  _tracking.reset(new xju::perception::Tracking());

  _cloud_pub_1 = nh.advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);
  _cloud_pub_2 = nh.advertise<sensor_msgs::PointCloud2>("non_ground_cloud", 1);
  _cloud_pub_3 = nh.advertise<sensor_msgs::PointCloud2>("db_scan", 1);
  _marker_pub_1 = nh.advertise<visualization_msgs::MarkerArray>("vel_marker", 1);
  _marker_pub_2 = nh.advertise<visualization_msgs::MarkerArray>("prism_box", 1);
  _marker_pub_3 = nh.advertise<visualization_msgs::MarkerArray>("cuboid_box", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/dense_points", 20);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 20);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> ASP;
  message_filters::Synchronizer<ASP> sync(ASP(20), cloud_sub, odom_sub);
  sync.registerCallback(boost::bind(&points_callback, _1, _2));

  ros::spin();
  return 0;
}
