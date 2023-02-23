//
// Created by tony on 2023/2/23.
//

#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>

namespace xju::pnc {
static const double Duration = 0.01;
class DynObjCtl {
public:
  DynObjCtl() {
    ros::NodeHandle nh;
    model_state_ = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    timer_ = nh.createTimer(ros::Duration(Duration),
                            [&](const ros::TimerEvent& e) {calPoseAndPub();});
  }

  ~DynObjCtl() = default;

private:
  void calPoseAndPub() {
    static double box_vel = 0.5 * Duration;
    static double cyl_vel = -0.6 * Duration;
    static double spe_vel = 0.8 * Duration;
    static double box_x = -5;
    static double cyl_x = -5;
    static double spe_x = -5;
    gazebo_msgs::ModelState msg;
    msg.model_name = "unit_box";
    msg.pose.position.y = 2;
    box_x += box_vel;
    inArea(box_x);
    msg.pose.position.x = box_x;
    model_state_.publish(msg);

    msg.model_name = "unit_cylinder";
    msg.pose.position.y = -3;
    cyl_x += cyl_vel;
    inArea(cyl_x);
    msg.pose.position.x = cyl_x;
    model_state_.publish(msg);

    msg.model_name = "unit_sphere";
    msg.pose.position.y = -5;
    spe_x += spe_vel;
    inArea(spe_x);
    msg.pose.position.x = spe_x;
    model_state_.publish(msg);
  }

  void inArea(double& val, double min = -5, double max = 5) {
    if (val < min) val = max;
    if (val > max) val = min;
  }

private:
  ros::Publisher model_state_;
  ros::Timer timer_;
};
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "xju_doc_node");

  xju::pnc::DynObjCtl _doc;
  ros::spin();

  return 0;
}