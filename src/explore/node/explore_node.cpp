//
// Created by tony on 2022/10/8.
//

#include "explore.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, xju::explore::NODE_NAME);
    xju::explore::Explore _explore;
    _explore.run();
    return 0;
}
