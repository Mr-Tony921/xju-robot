#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

struct TrackingObject {
std_msgs::Header header;
std::string label;
geometry_msgs::Pose pose;
sensor_msgs::PointCloud2 pointcloud;
geometry_msgs::Twist velocity;
geometry_msgs::Twist acceleration;
uint32_t id;
float angle;
geometry_msgs::Vector3 dimensions;
bool pose_reliable;
bool velocity_reliable;
uint8_t behavior_state; // FORWARD_STATE = 0, STOPPING_STATE = 1, BRANCH_LEFT_STATE = 2, BRANCH_RIGHT_STATE = 3, YIELDING_STATE = 4, ACCELERATING_STATE = 5, SLOWDOWN_STATE = 6
};

enum TrackingState : int {
Die = 0,        // No longer tracking
Init = 1,       // Start tracking
Stable = 4,     // Stable tracking
Occlusion = 5,  // Lost 1 frame possibly by occlusion
Lost = 10,      // About to lose target
};

enum MotionModel : int {
CV = 0,    // constant velocity
CTRV = 1,  // constant turn rate and velocity
RM = 2,    // random motion
};

class UKF {
  /*
  cv: Constant Velocity
  ctrv: Constatnt Turn Rate and Velocity
  rm: Random Motion
  */

public:
  int ukf_id_;

  int num_state_;

  int num_lidar_state_;

  int num_lidar_direction_state_;

  int num_motion_model_;

  //* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_merge_;

  //* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_cv_;

  //* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_ctrv_;

  //* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_rm_;

  //* state covariance matrix
  Eigen::MatrixXd p_merge_;

  //* state covariance matrix
  Eigen::MatrixXd p_cv_;

  //* state covariance matrix
  Eigen::MatrixXd p_ctrv_;

  //* state covariance matrix
  Eigen::MatrixXd p_rm_;

  //* predicted sigma points matrix
  Eigen::MatrixXd x_sig_pred_cv_;

  //* predicted sigma points matrix
  Eigen::MatrixXd x_sig_pred_ctrv_;

  //* predicted sigma points matrix
  Eigen::MatrixXd x_sig_pred_rm_;

  //* time when the state is true, in us
  long long time_;

  //* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_cv_;
  double std_a_ctrv_;
  double std_a_rm_;

  // CTRV
  double std_ctrv_yawdd_;
  // CV
  double std_cv_yawdd_;

  double std_rm_yawdd_;

  //* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  //* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  //* Weights of sigma points
  Eigen::VectorXd weights_c_;
  Eigen::VectorXd weights_s_;

  //* Sigma point spreading parameter
  double lambda_;

  double mode_match_prob_cv2cv_;
  double mode_match_prob_ctrv2cv_;
  double mode_match_prob_rm2cv_;

  double mode_match_prob_cv2ctrv_;
  double mode_match_prob_ctrv2ctrv_;
  double mode_match_prob_rm2ctrv_;

  double mode_match_prob_cv2rm_;
  double mode_match_prob_ctrv2rm_;
  double mode_match_prob_rm2rm_;

  double mode_match_prob_cv_;

  double mode_match_prob_ctrv_;

  double mode_match_prob_rm_;

  double mode_prob_cv_;
  double mode_prob_ctrv_;
  double mode_prob_rm_;

  std::vector<double> p1_;

  std::vector<double> p2_;

  std::vector<double> p3_;

  Eigen::VectorXd z_pred_cv_;
  Eigen::VectorXd z_pred_ctrv_;
  Eigen::VectorXd z_pred_rm_;

  Eigen::MatrixXd s_cv_;
  Eigen::MatrixXd s_ctrv_;
  Eigen::MatrixXd s_rm_;

  Eigen::MatrixXd k_cv_;
  Eigen::MatrixXd k_ctrv_;
  Eigen::MatrixXd k_rm_;

  double pd_;
  double pg_;

  int lifetime_;
  bool is_static_;

  // object msg information
  bool is_stable_;
  TrackingObject object_;
  std::string label_;
  double min_assiciation_distance_;

  // for env classification
  Eigen::VectorXd init_meas_;
  std::vector<double> vel_history_;

  double x_merge_yaw_;

  int tracking_num_;

  Eigen::VectorXd cv_meas_;
  Eigen::VectorXd ctrv_meas_;
  Eigen::VectorXd rm_meas_;

  Eigen::MatrixXd q_cv_;
  Eigen::MatrixXd q_ctrv_;
  Eigen::MatrixXd q_rm_;

  Eigen::MatrixXd r_cv_;
  Eigen::MatrixXd r_ctrv_;
  Eigen::MatrixXd r_rm_;

  double nis_cv_;
  double nis_ctrv_;
  double nis_rm_;

  Eigen::MatrixXd new_x_sig_cv_;
  Eigen::MatrixXd new_x_sig_ctrv_;
  Eigen::MatrixXd new_x_sig_rm_;

  Eigen::MatrixXd new_z_sig_cv_;
  Eigen::MatrixXd new_z_sig_ctrv_;
  Eigen::MatrixXd new_z_sig_rm_;

  Eigen::VectorXd new_z_pred_cv_;
  Eigen::VectorXd new_z_pred_ctrv_;
  Eigen::VectorXd new_z_pred_rm_;

  Eigen::MatrixXd new_s_cv_;
  Eigen::MatrixXd new_s_ctrv_;
  Eigen::MatrixXd new_s_rm_;

  // for lane direction combined filter
  bool is_direction_cv_available_;
  bool is_direction_ctrv_available_;
  bool is_direction_rm_available_;
  double std_lane_direction_;
  Eigen::MatrixXd lidar_direction_r_cv_;
  Eigen::MatrixXd lidar_direction_r_ctrv_;
  Eigen::MatrixXd lidar_direction_r_rm_;

  Eigen::VectorXd z_pred_lidar_direction_cv_;
  Eigen::VectorXd z_pred_lidar_direction_ctrv_;
  Eigen::VectorXd z_pred_lidar_direction_rm_;

  Eigen::MatrixXd s_lidar_direction_cv_;
  Eigen::MatrixXd s_lidar_direction_ctrv_;
  Eigen::MatrixXd s_lidar_direction_rm_;

  Eigen::MatrixXd k_lidar_direction_cv_;
  Eigen::MatrixXd k_lidar_direction_ctrv_;
  Eigen::MatrixXd k_lidar_direction_rm_;

  Eigen::VectorXd lidar_direction_ctrv_meas_;

  /**
   * Constructor
   */
  UKF();

  void updateYawWithHighProb();

  void initialize(const Eigen::VectorXd& z, const double timestamp,
                  const int target_ind);

  void updateModeProb(const std::vector<double>& lambda_vec);

  void mergeEstimationAndCovariance();

  void mixingProbability();

  void interaction();

  void predictionSUKF(const double dt, const bool has_subscribed_vectormap);

  void predictionIMMUKF(const double dt, const bool has_subscribed_vectormap);

  void findMaxZandS(Eigen::VectorXd& max_det_z, Eigen::MatrixXd& max_det_s);

  void updateMeasurementForCTRV(
    const std::vector<TrackingObject>& object_vec);

  void uppateForCTRV();

  void updateEachMotion(
    const double detection_probability, const double gate_probability,
    const double gating_thres,
    const std::vector<TrackingObject>& object_vec,
    std::vector<double>& lambda_vec);

  void updateSUKF(const std::vector<TrackingObject>& object_vec);

  void updateIMMUKF(
    const double detection_probability, const double gate_probability,
    const double gating_thres,
    const std::vector<TrackingObject>& object_vec);

  void ctrv(const double p_x, const double p_y, const double v,
            const double yaw, const double yawd, const double delta_t,
            std::vector<double>& state);

  void cv(const double p_x, const double p_y, const double v, const double yaw,
          const double yawd, const double delta_t, std::vector<double>& state);

  void randomMotion(const double p_x, const double p_y, const double v,
                    const double yaw, const double yawd, const double delta_t,
                    std::vector<double>& state);

  void initCovarQs(const double dt, const double yaw);

  void predictionMotion(const double delta_t, const int model_ind);

  void checkLaneDirectionAvailability(
    const TrackingObject& in_object,
    const double lane_direction_chi_thres, const bool use_sukf);

  void predictionLidarMeasurement(const int motion_ind,
                                  const int num_meas_state);

  double calculateNIS(const TrackingObject& in_object,
                      const int motion_ind);

  bool isLaneDirectionAvailable(const TrackingObject& in_object,
                                const int motion_ind,
                                const double lane_direction_chi_thres);

  // void updateKalmanGain(const int motion_ind, const int num_meas_state);
  void updateKalmanGain(const int motion_ind);

  double normalizeAngle(const double angle);

  void update(const bool use_sukf, const double detection_probability,
              const double gate_probability, const double gating_thres,
              const std::vector<TrackingObject>& object_vec);

  void prediction(const bool use_sukf, const bool has_subscribed_vectormap,
                  const double dt);
};
