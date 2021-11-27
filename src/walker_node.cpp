/**
 * @file walker_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <cstdlib>

#include "include/walker.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");
    ros::NodeHandle n;

    auto roomba = Walker();
    roomba.Vaccum();  // Call the main walker node

    ros::spin();
    return 0;
}
