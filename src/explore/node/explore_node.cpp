//
// Created by tony on 2022/10/8.
//

#include <csignal>
#include "explore.h"

std::shared_ptr<xju::explore::Explore> explore_ptr;

void sigintHandler(int sig) {
  if (explore_ptr) explore_ptr->pub_zero_vel();
  ROS_INFO("ros shutting down");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, xju::explore::NODE_NAME);
  explore_ptr = std::make_shared<xju::explore::Explore>();
  signal(SIGINT, sigintHandler);
  explore_ptr->run();
  explore_ptr.reset();
  return 0;
}
