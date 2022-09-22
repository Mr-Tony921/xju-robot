//
// Created by tony on 2022/9/22.
//

#include "xju_bridge.h"

int main (int argc, char** argv) {
  ros::init(argc, argv, xju::simu::NODE_NAME);
  xju::simu::XjuBridge _bridge_node;
  _bridge_node.init();

  ros::spin();
  return 0;
}
