//
// Created by tony on 2023/2/21.
//

#pragma once

#include <omp.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>

#include "ukf.h"

namespace xju::perception {

class RangeFilter {
public:
  RangeFilter() = default;

  ~RangeFilter() = default;

  pcl::PointCloud <pcl::PointXYZI> removePointsByRange(
    const pcl::PointCloud <pcl::PointXYZI>& cloud, double min_range, double max_range) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr narrowed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    narrowed_cloud_ptr->header = cloud.header;

    double square_min_range = min_range * min_range;
    double square_max_range = max_range * max_range;

#pragma omp for
    for (auto i = 0; i < cloud.size(); ++i) {
      pcl::PointXYZI p;
      p.x = cloud[i].x;
      p.y = cloud[i].y;
      p.z = cloud[i].z;
      p.intensity = cloud[i].intensity;
      double square_distance = p.x * p.x + p.y * p.y;

      if (square_min_range <= square_distance &&
          square_distance <= square_max_range) {
        narrowed_cloud_ptr->points.emplace_back(p);
      }
    }

    return *narrowed_cloud_ptr;
  }

  void removePointsInCar(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    pcl::ConditionalRemoval <pcl::PointXYZI> condrm;
    pcl::ConditionOr<pcl::PointXYZI>::Ptr car_range_cond(new pcl::ConditionOr <pcl::PointXYZI>);
    pcl::FieldComparison<pcl::PointXYZI>::ConstPtr x_cond_L(
      new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, -0.2));
    pcl::FieldComparison<pcl::PointXYZI>::ConstPtr x_cond_G(
      new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 0.2));
    pcl::FieldComparison<pcl::PointXYZI>::ConstPtr y_cond_L(
      new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, -0.2));
    pcl::FieldComparison<pcl::PointXYZI>::ConstPtr y_cond_G(
      new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, 0.2));
    car_range_cond->addComparison(x_cond_L);
    car_range_cond->addComparison(x_cond_G);
    car_range_cond->addComparison(y_cond_L);
    car_range_cond->addComparison(y_cond_G);

    pcl::ConditionAnd<pcl::PointXYZI>::Ptr car_full_range_cond(new pcl::ConditionAnd <pcl::PointXYZI>);
    car_full_range_cond->addCondition(car_range_cond);

    condrm.setCondition(car_full_range_cond);
    condrm.setKeepOrganized(false);

    condrm.setInputCloud(cloud);
    condrm.filter(*cloud);
  }
};

class GroundFilter {
#define CLIP_HEIGHT 1.0
#define MIN_DISTANCE 0.2
#define CONCENTRIC_DIVIDER_DISTANCE 0.01  // radial distance resolution
#define GENERAL_MAX_SLOPE 5 * M_PI / 180. // max slope with whole ground
#define LOCAL_MAX_SLOPE 8 * M_PI / 180.   // max slope between two poings
#define MIN_HEIGHT_THRES 0.05             // min height difference between two poings
#define RADIAL_DIVIDER_ANGLE 0.18         // radial angle resolution
#define RECLASS_DISTANCE_THRES 0.2        // reclassify threshold between two points
#define SENSOR_HEIGHT 0.68                // top laser scan height
public:
  GroundFilter() = default;

  ~GroundFilter() = default;

  void process(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
               pcl::PointCloud<pcl::PointXYZI>::Ptr& ground,
               pcl::PointCloud<pcl::PointXYZI>::Ptr& non_ground) {
    ground->clear();
    non_ground->clear();

    clipAbove(cloud, CLIP_HEIGHT);
    removeClosest(cloud, MIN_DISTANCE);

    PointCloudXYZIRT organized_points;
    std::vector<pcl::PointIndices> radial_division_indices;
    std::vector<PointCloudXYZIRT> radial_ordered_cloud;
    auto radial_dividers_num = std::ceil(360 / RADIAL_DIVIDER_ANGLE);
    XYZI2RTZ(cloud, radial_dividers_num, organized_points, radial_division_indices, radial_ordered_cloud);

    pcl::PointIndices ground_indices, non_ground_indices;
    separateGround(radial_ordered_cloud, ground_indices, non_ground_indices);

    pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
    extract_ground.setInputCloud(cloud);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
    extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
    extract_ground.filter(*ground);
    extract_ground.setNegative(true);
    extract_ground.filter(*non_ground);
  }

private:
  struct PointXYZIRT {
    pcl::PointXYZI point;

    float radius; // cylindric coords on XY plane
    float theta;  // angle deg on XY plane

    size_t radial_div;     //index of the radial division to which this point belongs to
    size_t concentric_div; // index of the concentric division to which this point belongs to
    size_t original_index; // index of this point in the source point cloud
  };

  typedef std::vector<PointXYZIRT> PointCloudXYZIRT;

  void clipAbove(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double clip_height) {
    pcl::ExtractIndices<pcl::PointXYZI> clipper;
    pcl::PointCloud<pcl::PointXYZI>::Ptr in(new pcl::PointCloud <pcl::PointXYZI>);
    pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(*cloud, *in);
    clipper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++) {
      if (in->points[i].z > clip_height) {
        indices.indices.emplace_back(i);
      }
    }
    clipper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    clipper.setNegative(true); //true to remove the indices
    cloud->clear();
    clipper.filter(*cloud);
  }

  void removeClosest(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double remove_distance) {
    pcl::ExtractIndices<pcl::PointXYZI> clipper;
    pcl::PointCloud<pcl::PointXYZI>::Ptr in(new pcl::PointCloud <pcl::PointXYZI>);
    pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(*cloud, *in);
    clipper.setInputCloud(in);
    pcl::PointIndices indices;
    auto r2 = remove_distance * remove_distance;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++) {
      double d2 = in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y;

      if (d2 < r2) {
        indices.indices.emplace_back(i);
      }
    }
    clipper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    clipper.setNegative(true); //true to remove the indices
    cloud->clear();
    clipper.filter(*cloud);
  }

  void XYZI2RTZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int dividers_num,
                PointCloudXYZIRT& out_organized_points,
                std::vector<pcl::PointIndices>& out_radial_divided_indices,
                std::vector<PointCloudXYZIRT>& out_radial_ordered_clouds) {
    using namespace std;
    out_organized_points.resize(cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(dividers_num);
    out_radial_ordered_clouds.resize(dividers_num);

    PointXYZIRT new_point;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      new_point.point = cloud->points[i];
      new_point.radius = hypotf(cloud->points[i].x, cloud->points[i].y);;
      new_point.theta = static_cast<float>(atan2(cloud->points[i].y, cloud->points[i].x) * 180 / M_PI);
      if (new_point.theta < 0) new_point.theta += 360;
      new_point.radial_div = static_cast<size_t>(floor(new_point.theta / RADIAL_DIVIDER_ANGLE));
      new_point.concentric_div = static_cast<size_t>(floor(fabs(new_point.radius / CONCENTRIC_DIVIDER_DISTANCE)));
      new_point.original_index = i;

      out_organized_points[i] = new_point;
      out_radial_divided_indices[new_point.radial_div].indices.emplace_back(i);
      out_radial_ordered_clouds[new_point.radial_div].emplace_back(new_point);

    }

#pragma omp for
    for (size_t i = 0; i < dividers_num; i++) {
      std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                [](const PointXYZIRT &a, const PointXYZIRT &b) { return a.radius < b.radius; });
    }
  }

  void separateGround(const std::vector<PointCloudXYZIRT>& in_radial_ordered_clouds,
                      pcl::PointIndices& out_ground_indices,
                      pcl::PointIndices& out_non_ground_indices) {
    using namespace std;
    out_ground_indices.indices.clear();
    out_non_ground_indices.indices.clear();
#pragma omp for
    for (auto i = 0; i < in_radial_ordered_clouds.size(); ++i) {
      float prev_radius = 0.f;
      float prev_height = -SENSOR_HEIGHT;
      bool prev_ground = false;
      bool current_ground = false;
      for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) {
        float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
        float height_threshold = tan(LOCAL_MAX_SLOPE) * points_distance;
        float current_height = in_radial_ordered_clouds[i][j].point.z;
        float general_height_threshold = tan(GENERAL_MAX_SLOPE) * in_radial_ordered_clouds[i][j].radius;

        //for points which are very close causing the height threshold to be tiny, set a minimum value
        if (height_threshold < MIN_HEIGHT_THRES) height_threshold = MIN_HEIGHT_THRES;

        //check current point height against the LOCAL threshold (previous point)
        if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold)) {
          //Check again using general geometry (radius from origin) if previous points wasn't ground
          if (!prev_ground) {
            current_ground = current_height <= (-SENSOR_HEIGHT + general_height_threshold) &&
                             current_height >= (-SENSOR_HEIGHT - general_height_threshold);
          } else {
            current_ground = true;
          }
        } else {
          //check if previous point is too far from previous one, if so classify again
          current_ground = points_distance > RECLASS_DISTANCE_THRES &&
                           (current_height <= (-SENSOR_HEIGHT + height_threshold) &&
                            current_height >= (-SENSOR_HEIGHT - height_threshold));
        }

        if (current_ground) {
          out_ground_indices.indices.emplace_back(in_radial_ordered_clouds[i][j].original_index);
          prev_ground = true;
        } else {
          out_non_ground_indices.indices.emplace_back(in_radial_ordered_clouds[i][j].original_index);
          prev_ground = false;
        }

        prev_radius = in_radial_ordered_clouds[i][j].radius;
        prev_height = in_radial_ordered_clouds[i][j].point.z;
      }
    }
  }
};

class Segment {
#define MINEPS 0.5
#define MINPTS 5
public:
  Segment() = default;

  ~Segment() = default;

  void dbScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr& remove_static_scan_ptr,
              std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters) {
    using namespace std;
    int C = 1;
    vector<point> dataset;
    float eps = MINEPS;
    int min_pts = MINPTS;
    for (auto const& p : remove_static_scan_ptr->points) {
      point pt(p.x, p.y, p.z, p.intensity);
      dataset.emplace_back(pt);
    }

    int core_flag;
    auto len = dataset.size();
    auto eps2 = eps * eps;
    for (auto i = 0; i < len; ++i) {
      for (auto j = i + 1; j < len; ++j) {
        if (squareDistance(dataset[i], dataset[j]) < eps2) {
          dataset[i].pts++;
          dataset[j].pts++;
          dataset[i].core_pts.emplace_back(j);
          dataset[j].core_pts.emplace_back(i);
        }
      }
    }

    for (auto i = 0; i < len; ++i) {
      if (dataset[i].visited == 1) continue;
      if (dataset[i].pts >= min_pts) {
        core_flag = 1;
        dataset[i].cluster = C;
        dataset[i].visited = 1;
        dataset[i].pointType = 1;
        point &p = dataset[i];
        expandCluster(dataset, p, C);
      } else {
        core_flag = 0;
        dataset[i].cluster = 0;
        dataset[i].pointType = 0;
      }
      C += core_flag;
    }

    vector<pcl::PointCloud<pcl::PointXYZI>> clusters_all;
    clusters_all.resize(C);
    pcl::PointXYZI a;
    for (auto i = 0; i < len; ++i) {
      if (dataset[i].cluster == 0) continue;
      a.x = dataset[i].x;
      a.y = dataset[i].y;
      a.z = dataset[i].z;
      a.intensity = dataset[i].intensity;
      clusters_all[dataset[i].cluster - 1].push_back(a);
    }

    for (auto& clt : clusters_all) {
      if (clt.points.size() > MINPTS) {
        clusters.emplace_back(clt);
      }
    }
  }

private:
  struct point {
  float x;
  float y;
  float z;
  float intensity;
  int cluster = 0;
  int pointType = 1;  // 0 noise  1 core
  int pts = 0;        // points in MinPts
  std::vector<int> core_pts;
  int visited = 0;

  point(float point_x, float point_y, float point_z, float point_intensity) {
    x = point_x;
    y = point_y;
    z = point_z;
    intensity = point_intensity;
  }
  };

  static float squareDistance(const point& a, const point& b) {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
  }

  static void expandCluster(std::vector<point>& dataset, point& data, int c) {
    for (auto i = 0; i < data.pts; ++i) {
      if (dataset[data.core_pts[i]].visited != 0) continue;

      dataset[data.core_pts[i]].visited = 1;
      dataset[data.core_pts[i]].cluster = c;
      dataset[data.core_pts[i]].pointType = 1;
      expandCluster(dataset, dataset[data.core_pts[i]], c);
    }
  }
};

struct TrackingObjectArray {
std_msgs::Header header;
std::vector<TrackingObject> objects;
};

class Tracking {
  const double CENTROID_DISTANCE = 0.2;
public:
  Tracking() = default;

  ~Tracking() = default;

  void generateTrackingData(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusters,
                            double timestamp, uint32_t sequence) {
    input_tracking_data_.objects.clear();
    input_tracking_data_.header.stamp.fromSec(timestamp);
    input_tracking_data_.header.seq = sequence;
    input_tracking_data_.header.frame_id = "top_laser_link";

    Eigen::Vector4f centroid;
    TrackingObject obj;
    obj.label = "Obstacle";
    obj.pose.orientation.w = 1.0;
    sensor_msgs::PointCloud2 pc2;
    for (auto const& cluster : clusters) {
      pcl::compute3DCentroid(cluster, centroid);
      obj.pose.position.x = centroid[0];
      obj.pose.position.y = centroid[1];
      obj.pose.position.z = centroid[2];
      pcl::toROSMsg(cluster, pc2);
      pc2.header = input_tracking_data_.header;
      obj.pointcloud = pc2;

      input_tracking_data_.objects.emplace_back(obj);
    }
  }

  TrackingObjectArray process() {
    // todo: Should unitize coordinate system first, while we simply ignored -.-||
    double timestamp = input_tracking_data_.header.stamp.toSec();
    std::vector<bool> matching_vec(input_tracking_data_.objects.size(), false);

    TrackingObjectArray result;
    if (input_tracking_data_.objects.empty()) return result;

    if (!init_) {
      initTracker(timestamp);
      makeOutput(matching_vec, result);
      return result;
    }

    double dt = (timestamp - time_);
    time_ = timestamp;

    // start UKF process
    for (auto & target : targets_) {
      target.is_stable_ = false;
      target.is_static_ = false;

      if (target.tracking_num_ == TrackingState::Die) continue;

      // prevent ukf not to explode
      if (target.p_merge_.determinant() > prevent_explosion_threshold_ ||
          target.p_merge_(4, 4) > prevent_explosion_threshold_) {
        target.tracking_num_ = TrackingState::Die;
        continue;
      }

      target.prediction(use_sukf_, has_subscribed_vectormap_, dt);

      std::vector<TrackingObject> object_vec;
      if (!probabilisticDataAssociation(dt, matching_vec, object_vec, target)) continue;

      target.update(use_sukf_, detection_probability_, gate_probability_, gating_threshold_, object_vec);
    }
    // end UKF process

    // making new ukf target for no data association objects
    makeNewTargets(timestamp, matching_vec);

    // static dynamic classification
    staticClassification();

    // making output for visualization
    makeOutput(matching_vec, result);

    // remove unnecessary ukf object
    removeUnnecessaryTarget();

    return result;
  }

private:
  TrackingObjectArray input_tracking_data_;
  int target_id_ = 0;
  int life_time_thres_ = 8;
  int static_num_history_thres_ = 3;
  bool init_ = false;
  bool use_sukf_ = true;
  bool has_subscribed_vectormap_ = false;
  double time_ = 0.0;
  double merge_distance_threshold_= 0.5;
  double prevent_explosion_threshold_ =1000;
  double static_velocity_thres_ = 0.5;
  double gating_threshold_ = 9.22;
  double detection_probability_ = 0.9;
  double gate_probability_ = 0.99;
  std::vector<UKF> targets_ {};

  void initTracker(double timestamp) {
    for (auto const& obj : input_tracking_data_.objects) {
      double px = obj.pose.position.x;
      double py = obj.pose.position.y;
      Eigen::VectorXd init_meas = Eigen::VectorXd(2);
      init_meas << px, py;

      UKF ukf;
      ukf.initialize(init_meas, timestamp, target_id_);
      targets_.emplace_back(ukf);
      ++target_id_;
    }

    time_ = timestamp;
    init_ = true;
  }

  void secondInit(UKF& target, const std::vector<TrackingObject>& object_vec, double dt) {
    if (object_vec.empty()) {
      target.tracking_num_ = TrackingState::Die;
      return;
    }

    // record init measurement for env classification
    target.init_meas_ << target.x_merge_(0), target.x_merge_(1);

    // state update
    double target_x = object_vec[0].pose.position.x;
    double target_y = object_vec[0].pose.position.y;
    double target_diff_x = target_x - target.x_merge_(0);
    double target_diff_y = target_y - target.x_merge_(1);
    double target_yaw = atan2(target_diff_y, target_diff_x);
    double dist = std::hypot(target_diff_x, target_diff_y);
    double target_v = dist / dt;

    while (target_yaw > M_PI) target_yaw -= 2. * M_PI;
    while (target_yaw < -M_PI) target_yaw += 2. * M_PI;

    target.x_merge_(0) = target.x_cv_(0) = target.x_ctrv_(0) = target.x_rm_(0) = target_x;
    target.x_merge_( 1) = target.x_cv_(1) = target.x_ctrv_(1) = target.x_rm_(1) = target_y;
    target.x_merge_(2) = target.x_cv_(2) = target.x_ctrv_(2) = target.x_rm_(2) = target_v;
    target.x_merge_(3) = target.x_cv_(3) = target.x_ctrv_(3) = target.x_rm_(3) = target_yaw;
    ++target.tracking_num_;
  }

  void makeNewTargets(double timestamp, const std::vector<bool>& matching_vec) {
    for (size_t i = 0; i < input_tracking_data_.objects.size(); ++i) {
      if (!matching_vec[i]) {
        double px = input_tracking_data_.objects[i].pose.position.x;
        double py = input_tracking_data_.objects[i].pose.position.y;
        Eigen::VectorXd init_meas = Eigen::VectorXd(2);
        init_meas << px, py;

        UKF ukf;
        ukf.initialize(init_meas, timestamp, target_id_);
        ukf.object_ = input_tracking_data_.objects[i];
        targets_.emplace_back(ukf);
        ++target_id_;
      }
    }
  }

  void makeOutput(const std::vector<bool>& matching_vec, TrackingObjectArray& detected_objects_output) {
    TrackingObjectArray tmp_objects;
    tmp_objects.header = input_tracking_data_.header;
    std::vector<size_t> used_targets_indices;

    for (size_t i = 0; i < targets_.size(); ++i) {
      double tx = targets_[i].x_merge_(0);
      double ty = targets_[i].x_merge_(1);

      double tv = targets_[i].x_merge_(2);
      double tyaw = targets_[i].x_merge_(3);
      double tyaw_rate = targets_[i].x_merge_(4);

      while (tyaw > M_PI) tyaw -= 2. * M_PI;
      while (tyaw < -M_PI) tyaw += 2. * M_PI;

      tf::Quaternion q = tf::createQuaternionFromYaw(tyaw);

      TrackingObject dd;
      dd = targets_[i].object_;
      dd.id = targets_[i].ukf_id_;
      dd.velocity.linear.x = tv;
      dd.acceleration.linear.y = tyaw_rate;
      dd.velocity_reliable = targets_[i].is_stable_;
      dd.pose_reliable = targets_[i].is_stable_;

      if (!targets_[i].is_static_ && targets_[i].is_stable_) {
        // Aligh the longest side of dimentions with the estimated orientation
        if (targets_[i].object_.dimensions.x < targets_[i].object_.dimensions.y) {
          dd.dimensions.x = targets_[i].object_.dimensions.y;
          dd.dimensions.y = targets_[i].object_.dimensions.x;
        }

        dd.pose.position.x = tx;
        dd.pose.position.y = ty;

        if (!std::isnan(q[0])) dd.pose.orientation.x = q[0];
        if (!std::isnan(q[1])) dd.pose.orientation.y = q[1];
        if (!std::isnan(q[2])) dd.pose.orientation.z = q[2];
        if (!std::isnan(q[3])) dd.pose.orientation.w = q[3];
      }
      updateBehaviorState(targets_[i], use_sukf_, dd);

      if (targets_[i].is_stable_ ||
          (targets_[i].tracking_num_ >= TrackingState::Init &&
           targets_[i].tracking_num_ < TrackingState::Stable)) {
        tmp_objects.objects.emplace_back(dd);
        used_targets_indices.emplace_back(i);
      }
    }

    detected_objects_output = removeRedundantObjects(tmp_objects, used_targets_indices);
  }

  void updateBehaviorState(const UKF& target, const bool use_sukf, TrackingObject& object) {
    if (use_sukf) {
      object.behavior_state = MotionModel::CTRV;
    } else if (target.mode_prob_cv_ > target.mode_prob_ctrv_ &&
               target.mode_prob_cv_ > target.mode_prob_rm_) {
      object.behavior_state = MotionModel::CV;
    } else if (target.mode_prob_ctrv_ > target.mode_prob_cv_ &&
               target.mode_prob_ctrv_ > target.mode_prob_rm_) {
      object.behavior_state = MotionModel::CTRV;
    } else {
      object.behavior_state = MotionModel::RM;
    }
  }

  bool probabilisticDataAssociation(double dt, std::vector<bool>& matching_vec,
                                    std::vector<TrackingObject>& object_vec, UKF& target) {
    double det_s = 0;
    Eigen::VectorXd max_det_z;
    Eigen::MatrixXd max_det_s;
    bool success = true;

    if (use_sukf_) {
      max_det_z = target.z_pred_ctrv_;
      max_det_s = target.s_ctrv_;
      det_s = max_det_s.determinant();
    } else {
      // find maxDetS associated with predZ
      target.findMaxZandS(max_det_z, max_det_s);
      det_s = max_det_s.determinant();
    }

    // prevent ukf not to explode
    if (std::isnan(det_s) || det_s > prevent_explosion_threshold_) {
      target.tracking_num_ = TrackingState::Die;
      success = false;
      return success;
    }

    bool is_second_init = target.tracking_num_ == TrackingState::Init;

    // measurement gating
    measurementValidation(target, is_second_init, max_det_z, max_det_s, object_vec, matching_vec);

    // second detection for a target: update v and yaw
    if (is_second_init) {
      secondInit(target, object_vec, dt);
      success = false;
      return success;
    }

    updateTargetWithAssociatedObject(object_vec, target);

    if (target.tracking_num_ == TrackingState::Die) {
      success = false;
      return success;
    }

    return success;
  }

  void measurementValidation(UKF& target, bool second_init,
                             const Eigen::VectorXd& max_det_z,
                             const Eigen::MatrixXd& max_det_s,
                             std::vector<TrackingObject>& object_vec,
                             std::vector<bool>& matching_vec) {
    bool exists_smallest_nis_object = false;
    double smallest_nis = std::numeric_limits<double>::max();
    int smallest_nis_ind = 0;

    for (size_t i = 0; i < input_tracking_data_.objects.size(); ++i) {
      double x = input_tracking_data_.objects[i].pose.position.x;
      double y = input_tracking_data_.objects[i].pose.position.y;

      Eigen::VectorXd meas = Eigen::VectorXd(2);
      meas << x, y;

      Eigen::VectorXd diff = meas - max_det_z;
      double nis = diff.transpose() * max_det_s.inverse() * diff;

      if (nis < gating_threshold_) {
        if (nis < smallest_nis) {
          smallest_nis = nis;
          target.object_ = input_tracking_data_.objects[i];
          smallest_nis_ind = i;
          exists_smallest_nis_object = true;
        }
      }
    }

    if (exists_smallest_nis_object) {
      matching_vec[smallest_nis_ind] = true;
      object_vec.emplace_back(target.object_);
    }
  }

  void updateTargetWithAssociatedObject(const std::vector<TrackingObject>& object_vec, UKF& target) {
    ++target.lifetime_;
    if (!target.object_.label.empty() && target.object_.label != "unknown") {
      target.label_ = target.object_.label;
    }
    updateTrackingNum(object_vec, target);
    if (target.tracking_num_ == TrackingState::Stable ||
        target.tracking_num_ == TrackingState::Occlusion) {
      target.is_stable_ = true;
    }
  }

  void updateTrackingNum(const std::vector<TrackingObject>& object_vec, UKF& target) {
    if (!object_vec.empty()) {
      if (target.tracking_num_ < TrackingState::Stable) {
        ++target.tracking_num_;
      } else if (target.tracking_num_ == TrackingState::Stable) {
        target.tracking_num_ = TrackingState::Stable;
      } else if (target.tracking_num_ >= TrackingState::Stable &&
                 target.tracking_num_ < TrackingState::Lost) {
        target.tracking_num_ = TrackingState::Stable;
      } else if (target.tracking_num_ == TrackingState::Lost) {
        target.tracking_num_ = TrackingState::Die;
      }
    } else {
      if (target.tracking_num_ < TrackingState::Stable) {
        target.tracking_num_ = TrackingState::Die;
      } else if (target.tracking_num_ >= TrackingState::Stable &&
                 target.tracking_num_ < TrackingState::Lost) {
        ++target.tracking_num_;
      } else if (target.tracking_num_ == TrackingState::Lost) {
        target.tracking_num_ = TrackingState::Die;
      }
    }
  }

  void staticClassification() {
    for (auto& target : targets_) {
      // targets_[i].x_merge_(2) is referred for estimated velocity
      double current_velocity = std::abs(target.x_merge_(2));
      target.vel_history_.emplace_back(current_velocity);
      if (target.tracking_num_ == TrackingState::Stable && target.lifetime_ > life_time_thres_) {
        int index = 0;
        double sum_vel = 0;
        double avg_vel = 0;
        for (auto rit = target.vel_history_.rbegin(); index < static_num_history_thres_; ++rit) {
          ++index;
          sum_vel += *rit;
        }

        avg_vel = double(sum_vel / static_num_history_thres_);
        if (avg_vel < static_velocity_thres_ && current_velocity < static_velocity_thres_) {
          target.is_static_ = true;
        }
      }
    }
  }

  TrackingObjectArray removeRedundantObjects(
    const TrackingObjectArray& in_detected_objects,
    const std::vector<size_t>& in_tracker_indices) {
    if (in_detected_objects.objects.size() != in_tracker_indices.size()) return in_detected_objects;

    TrackingObjectArray resulting_objects;
    resulting_objects.header = in_detected_objects.header;

    std::vector<geometry_msgs::Point> centroids;
    // create unique points
    for (const auto & object : in_detected_objects.objects) {
      if (!isPointInPool(centroids, object.pose.position)) {
        centroids.emplace_back(object.pose.position);
      }
    }
    // assign objects to the points
    std::vector<std::vector<size_t>> matching_objects(centroids.size());
    for (size_t k = 0; k < in_detected_objects.objects.size(); k++) {
      const auto& object = in_detected_objects.objects[k];
      for (size_t i = 0; i < centroids.size(); i++) {
        if (arePointsClose(object.pose.position, centroids[i], merge_distance_threshold_)) {
          matching_objects[i].emplace_back(k);  // store index of matched object to this point
        }
      }
    }
    // get oldest object on each point
    for (auto & matching_object : matching_objects) {
      size_t oldest_object_index = 0;
      int oldest_lifespan = -1;
      std::string best_label;
      for (unsigned long current_index : matching_object) {
        int current_lifespan = targets_[in_tracker_indices[current_index]].lifetime_;
        if (current_lifespan > oldest_lifespan) {
          oldest_lifespan = current_lifespan;
          oldest_object_index = current_index;
        }
        if (!targets_[in_tracker_indices[current_index]].label_.empty() &&
            targets_[in_tracker_indices[current_index]].label_ != "unknown") {
          best_label = targets_[in_tracker_indices[current_index]].label_;
        }
      }
      // delete nearby targets except for the oldest target
      for (unsigned long current_index : matching_object) {
        if (current_index != oldest_object_index) {
          targets_[in_tracker_indices[current_index]].tracking_num_ = TrackingState::Die;
        }
      }
      TrackingObject best_object;
      best_object = in_detected_objects.objects[oldest_object_index];
      if (best_label != "unknown" && !best_label.empty()) {
        best_object.label = best_label;
      }

      resulting_objects.objects.emplace_back(best_object);
    }

    return resulting_objects;
  }

  void removeUnnecessaryTarget() {
    for (auto it = targets_.begin(); it != targets_.end(); ) {
      if (it->tracking_num_ == TrackingState::Die) {
        it = targets_.erase(it);
      } else {
        ++it;
      }
    }
  }

  bool arePointsClose(const geometry_msgs::Point& in_point_a,
                      const geometry_msgs::Point& in_point_b,
                      float in_radius) {
    return (fabs(in_point_a.x - in_point_b.x) <= in_radius) &&
           (fabs(in_point_a.y - in_point_b.y) <= in_radius);
  }

  bool arePointsEqual(const geometry_msgs::Point& in_point_a,
                      const geometry_msgs::Point& in_point_b) {
    return arePointsClose(in_point_a, in_point_b, CENTROID_DISTANCE);
  }

  bool isPointInPool(const std::vector<geometry_msgs::Point>& in_pool,
                     const geometry_msgs::Point& in_point) {
    for (const auto & ele : in_pool) {
      if (arePointsEqual(ele, in_point)) return true;
    }
    return false;
  }
};

class Vertex {
public:
  double x;
  double y;

  Vertex() {}

  Vertex(double a, double b) {
    x = a;
    y = b;
  }

  bool operator<(const Vertex& p) const {
    return x < p.x || (x == p.x && y < p.y);
  }

  Vertex operator+(const Vertex& p) const {
    Vertex vertex;
    vertex.x = x + p.x;
    vertex.y = y + p.y;
    return vertex;
  }

  Vertex operator-(const Vertex& p) const {
    Vertex vertex;
    vertex.x = x - p.x;
    vertex.y = y - p.y;
    return vertex;
  }

  friend Vertex operator*(const double& k, const Vertex& p) {
    Vertex vertex;
    vertex.x = k * p.x;
    vertex.y = k * p.y;
    return vertex;
  }
};

class Prism {
public:
  std::vector<Vertex> vertices;
  double top;
  double bottom;
};

class Prisms {
public:
  std::vector<Prism> prisms_;
  ros::Time time;
  visualization_msgs::MarkerArray toRviz() {
    auto box_num = prisms_.size();
    visualization_msgs::MarkerArray multi_bbox;
    multi_bbox.markers.resize(box_num + 1);

    multi_bbox.markers[0].header.frame_id = "top_laser_link";
    multi_bbox.markers[0].header.stamp = Prisms::time;
    multi_bbox.markers[0].action = visualization_msgs::Marker::DELETEALL;

    for (auto i = 0; i < box_num; ++i) {
      multi_bbox.markers[i + 1].header.frame_id = "top_laser_link";
      multi_bbox.markers[i + 1].header.stamp = Prisms::time;
      multi_bbox.markers[i + 1].action = visualization_msgs::Marker::ADD;
      multi_bbox.markers[i + 1].pose.orientation.w = 1.0;
      multi_bbox.markers[i + 1].id = i;
      multi_bbox.markers[i + 1].type = visualization_msgs::Marker::LINE_STRIP;

      multi_bbox.markers[i + 1].scale.x = 0.1;
      multi_bbox.markers[i + 1].color.r = 0.0f;
      multi_bbox.markers[i + 1].color.g = 0.0f;
      multi_bbox.markers[i + 1].color.b = 1.0f;
      multi_bbox.markers[i + 1].color.a = 1.0;

      auto vertices_size = prisms_[i].vertices.size();
      multi_bbox.markers[i + 1].points.resize(vertices_size * 5);
      for (int j = 0; j < vertices_size; j++) {
        multi_bbox.markers[i + 1].points[5 * j].x = prisms_[i].vertices[j % vertices_size].x;
        multi_bbox.markers[i + 1].points[5 * j].y = prisms_[i].vertices[j % vertices_size].y;
        multi_bbox.markers[i + 1].points[5 * j].z = prisms_[i].top;
        multi_bbox.markers[i + 1].points[5 * j + 1].x = prisms_[i].vertices[j % vertices_size].x;
        multi_bbox.markers[i + 1].points[5 * j + 1].y = prisms_[i].vertices[j % vertices_size].y;
        multi_bbox.markers[i + 1].points[5 * j + 1].z = prisms_[i].bottom;
        multi_bbox.markers[i + 1].points[5 * j + 2].x = prisms_[i].vertices[(j + 1) % vertices_size].x;
        multi_bbox.markers[i + 1].points[5 * j + 2].y = prisms_[i].vertices[(j + 1) % vertices_size].y;
        multi_bbox.markers[i + 1].points[5 * j + 2].z = prisms_[i].bottom;
        multi_bbox.markers[i + 1].points[5 * j + 3].x = prisms_[i].vertices[(j + 1) % vertices_size].x;
        multi_bbox.markers[i + 1].points[5 * j + 3].y = prisms_[i].vertices[(j + 1) % vertices_size].y;
        multi_bbox.markers[i + 1].points[5 * j + 3].z = prisms_[i].top;
        multi_bbox.markers[i + 1].points[5 * j + 4].x = prisms_[i].vertices[j % vertices_size].x;
        multi_bbox.markers[i + 1].points[5 * j + 4].y = prisms_[i].vertices[j % vertices_size].y;
        multi_bbox.markers[i + 1].points[5 * j + 4].z = prisms_[i].top;
      }
    }

    return multi_bbox;
  }
};

class ConvexHull {
public:
  std::vector<Vertex> origin_points_;
  std::vector<Vertex> vertices_;
  ConvexHull(std::vector<Vertex>& P) {
    origin_points_ = P;
    size_t n = P.size(), k = 0;
    if (n <= 3) {
      vertices_.assign(P.begin(), P.end());
      return;
    }

    vertices_.resize(2*n);

    std::sort(P.begin(), P.end());

    // Build lower hull
    for (size_t i = 0; i < n; ++i) {
      while (k >= 2 && cross(vertices_[k-2], vertices_[k-1], P[i]) <= 0) k--;
      vertices_[k++] = P[i];
    }

    // Build upper hull
    for (size_t i = n-1, t = k+1; i > 0; --i) {
      while (k >= t && cross(vertices_[k-2], vertices_[k-1], P[i-1]) <= 0) k--;
      vertices_[k++] = P[i-1];
    }

    vertices_.resize(k-1);
  }

  std::vector<Vertex> toRec2() {
    using namespace std;
    vector<double> rectangles_weight;
    vector<vector<Vertex>> rectangles_points;
    for (auto i = 0; i < vertices_.size(); ++i) {
      if (i != vertices_.size() - 1) {
        double k1;
        if (vertices_[i + 1].x != vertices_[i].x && vertices_[i + 1].y != vertices_[i].y) {
          k1 = (vertices_[i + 1].y - vertices_[i].y) / (vertices_[i + 1].x - vertices_[i].x);
        } else if (vertices_[i + 1].x == vertices_[i].x && vertices_[i + 1].y == vertices_[i].y) {
          k1 = (vertices_[i + 1].y + 0.0001 - vertices_[i].y) / (vertices_[i + 1].x + 0.0001 - vertices_[i].x);
        } else if (vertices_[i + 1].x == vertices_[i].x) {
          k1 = (vertices_[i + 1].y - vertices_[i].y) / (vertices_[i + 1].x + 0.0001 - vertices_[i].x);
        } else if (vertices_[i + 1].y == vertices_[i].y) {
          k1 = (vertices_[i + 1].y + 0.0001 - vertices_[i].y) / (vertices_[i + 1].x - vertices_[i].x);
        }
        double b11 = vertices_[i].y - k1 * vertices_[i].x;
        double maxdis1 = 0;
        int dispoint = i;
        for (auto j = 0; j < vertices_.size(); ++j) {
          double dis = abs((k1 * vertices_[j].x - vertices_[j].y + b11) / sqrt(k1 * k1 + 1));
          if (dis > maxdis1) {
            maxdis1 = dis;
            dispoint = j;
          }
        }
        double b12 = vertices_[dispoint].y - k1 * vertices_[dispoint].x;
        double k2 = -1 / k1;
        double maxdis2 = 0;
        double mindis2 = 0;
        int maxdispoint = i;
        int mindispoint = i;
        double b1_ = vertices_[i].y - k2 * vertices_[i].x;
        for (auto j = 0; j < vertices_.size(); ++j) {
          double dis = (k2 * vertices_[j].x - vertices_[j].y + b1_) / sqrt(k2 * k2 + 1);
          if (dis > maxdis2) {
            maxdis2 = dis;
            maxdispoint = j;
          } else if (dis < mindis2) {
            mindis2 = dis;
            mindispoint = j;
          }
        }
        double b21 = vertices_[maxdispoint].y - k2 * vertices_[maxdispoint].x;
        double b22 = vertices_[mindispoint].y - k2 * vertices_[mindispoint].x;

        double point_weight = 0;
        for (int j = 0; j < origin_points_.size(); j++) {
          double minweight1 = min(abs((k1 * origin_points_[j].x - origin_points_[j].y + b11) / sqrt(k1 * k1 + 1)),
                                  abs((k1 * origin_points_[j].x - origin_points_[j].y + b12) / sqrt(k1 * k1 + 1)));
          double minweight2 = min(abs((k2 * origin_points_[j].x - origin_points_[j].y + b21) / sqrt(k2 * k2 + 1)),
                                  abs((k2 * origin_points_[j].x - origin_points_[j].y + b22) / sqrt(k2 * k2 + 1)));
          point_weight = point_weight + min(minweight1, minweight2);
        }
        point_weight = point_weight / origin_points_.size();
        rectangles_weight.emplace_back(point_weight);

        vector<Vertex> vertices;
        Vertex point1;
        point1.x = (b21 - b11) / (k1 - k2);
        point1.y = (k1 * b21 - k2 * b11) / (k1 - k2);
        vertices.emplace_back(point1);
        Vertex point2;
        point2.x = (b21 - b12) / (k1 - k2);
        point2.y = (k1 * b21 - k2 * b12) / (k1 - k2);
        vertices.emplace_back(point2);
        Vertex point3;
        point3.x = (b22 - b12) / (k1 - k2);
        point3.y = (k1 * b22 - k2 * b12) / (k1 - k2);
        vertices.emplace_back(point3);
        Vertex point4;
        point4.x = (b22 - b11) / (k1 - k2);
        point4.y = (k1 * b22 - k2 * b11) / (k1 - k2);
        vertices.emplace_back(point4);

        rectangles_points.emplace_back(vertices);
      } else if (i == vertices_.size() - 1) {
        double k1;
        if (vertices_[i + 1].x != vertices_[i].x && vertices_[i + 1].y != vertices_[i].y) {
          k1 = (vertices_[i + 1].y - vertices_[i].y) / (vertices_[i + 1].x - vertices_[i].x);
        } else if (vertices_[i + 1].x == vertices_[i].x && vertices_[i + 1].y == vertices_[i].y) {
          k1 = (vertices_[i + 1].y + 0.0001 - vertices_[i].y) / (vertices_[i + 1].x + 0.0001 - vertices_[i].x);
        } else if (vertices_[i + 1].x == vertices_[i].x) {
          k1 = (vertices_[i + 1].y - vertices_[i].y) / (vertices_[i + 1].x + 0.0001 - vertices_[i].x);
        } else if (vertices_[i + 1].y == vertices_[i].y) {
          k1 = (vertices_[i + 1].y + 0.0001 - vertices_[i].y) / (vertices_[i + 1].x - vertices_[i].x);
        }
        double b11 = vertices_[i].y - k1 * vertices_[i].x;
        double maxdis1 = 0;
        int dispoint = i;
        for (int j = 0; j < vertices_.size(); j++) {
          double dis = abs((k1 * vertices_[j].x - vertices_[j].y + b11) / sqrt(k1 * k1 + 1));
          if (dis > maxdis1) {
            maxdis1 = dis;
            dispoint = j;
          }
        }
        double b12 = vertices_[dispoint].y - k1 * vertices_[dispoint].x;
        double k2 = -1 / k1;
        double maxdis2 = 0;
        double mindis2 = 0;
        int maxdispoint = i;
        int mindispoint = i;
        double b1_ = vertices_[i].y - k2 * vertices_[i].x;
        for (int j = 0; j < vertices_.size(); j++) {
          double dis = (k2 * vertices_[j].x - vertices_[j].y + b1_) / sqrt(k2 * k2 + 1);
          if (dis > maxdis2) {
            maxdis2 = dis;
            maxdispoint = j;
          } else if (dis < mindis2) {
            mindis2 = dis;
            mindispoint = j;
          }
        }
        double b21 = vertices_[maxdispoint].y - k2 * vertices_[maxdispoint].x;
        double b22 = vertices_[mindispoint].y - k2 * vertices_[mindispoint].x;

        double point_weight = 0;
        for (auto & origin_point : origin_points_) {
          double minweight1 = min(abs((k1 * origin_point.x - origin_point.y + b11) / sqrt(k1 * k1 + 1)),
                                  abs((k1 * origin_point.x - origin_point.y + b12) / sqrt(k1 * k1 + 1)));
          double minweight2 = min(abs((k2 * origin_point.x - origin_point.y + b21) / sqrt(k2 * k2 + 1)),
                                  abs((k2 * origin_point.x - origin_point.y + b22) / sqrt(k2 * k2 + 1)));
          point_weight = point_weight + min(minweight1, minweight2);
        }
        point_weight = point_weight / origin_points_.size();
        rectangles_weight.emplace_back(point_weight);

        std::vector<Vertex> vertices;
        Vertex point1;
        point1.x = (b21 - b11) / (k1 - k2);
        point1.y = (k1 * b21 - k2 * b11) / (k1 - k2);
        vertices.emplace_back(point1);
        Vertex point2;
        point2.x = (b21 - b12) / (k1 - k2);
        point2.y = (k1 * b21 - k2 * b12) / (k1 - k2);
        vertices.emplace_back(point2);
        Vertex point3;
        point3.x = (b22 - b12) / (k1 - k2);
        point3.y = (k1 * b22 - k2 * b12) / (k1 - k2);
        vertices.emplace_back(point3);
        Vertex point4;
        point4.x = (b22 - b11) / (k1 - k2);
        point4.y = (k1 * b22 - k2 * b11) / (k1 - k2);
        vertices.emplace_back(point4);

        rectangles_points.emplace_back(vertices);
      }
    }

    double minrectangles = 10000;
    int num = -1;
    for (int i = 0; i < rectangles_weight.size(); i++) {
      if (rectangles_weight[i] < minrectangles) {
        minrectangles = rectangles_weight[i];
        num = i;
      }
    }
    std::vector<Vertex> rect1 = rectangles_points[num];
    return rect1;
  }

private:
  double cross(const Vertex &O, const Vertex &A, const Vertex &B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
  }
};

class BoundingBox {
public:
  Prisms prisms_;
  Prisms cuboids_;
  BoundingBox(TrackingObjectArray& input, ros::Time time) {
    for (auto i = 0; i < input.objects.size(); ++i) {
      if (!input.objects[i].pointcloud.data.empty()) {
        std::vector<Vertex> point_input;

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(input.objects[i].pointcloud, *scan);

        for (auto const& pt : *scan) {
          point_input.emplace_back(pt.x, pt.y);
        }
        ConvexHull* convex_hull(new ConvexHull(point_input));

        pcl::PointXYZI min_pt;
        pcl::PointXYZI max_pt;
        pcl::getMinMax3D(*scan, min_pt, max_pt);

        Prism prism;
        prism.vertices = convex_hull->vertices_;
        prism.bottom = min_pt.z;
        prism.top = max_pt.z;
        prisms_.prisms_.emplace_back(prism);
        prisms_.time = time;

        prism.vertices = convex_hull->toRec2();
        cuboids_.prisms_.emplace_back(prism);
        cuboids_.time = time;
      }
    }
  }
};

}
