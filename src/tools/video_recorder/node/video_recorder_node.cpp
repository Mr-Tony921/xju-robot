//
// Created by tony on 2023/6/8.
//

#include <csignal>
#include "video_recorder.h"

std::shared_ptr<xju::rec::VideoRecorder> vr_ptr;

void sigintHandler(int sig) {
  if (vr_ptr) vr_ptr->finish_recorder();
  ROS_INFO("ros shutting down");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, xju::rec::NODE_NAME);
  vr_ptr = std::make_shared<xju::rec::VideoRecorder>();
  signal(SIGINT, sigintHandler);
  vr_ptr->start_recorder();
  vr_ptr.reset();
  return 0;
}
