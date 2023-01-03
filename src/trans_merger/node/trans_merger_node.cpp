//
// Created by tony on 2023/1/3.
//

#include "trans_merger.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "general_sensor_processor");
  xju::trans_merger::TransMerger _trans_merger;
  ros::spin();
  return 0;
}
