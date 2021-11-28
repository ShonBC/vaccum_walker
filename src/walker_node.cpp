/**
 * @file walker_node.cpp
 * @author your name (you@domain.com)
 * @brief Main Vaccum_walker node. Turtlebot will drive forward until an obstacle is detected then turn and resume driving forward again.
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/walker.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");
    ros::NodeHandle n;

    Walker roomba;
    roomba.Vaccum(n);  // Call the main walker node

    ros::spin();
    return 0;
}
