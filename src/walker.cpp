/**
 * @file walker.h
 * @author Shon Cortes (scortes3@umd.edu)
 * @brief Implementation of a simple walker algorithm much like a Roomba robot vacuum cleaner. The robot should move forward until it reaches an obstacle (but not colliding), then rotate in place until the way ahead is clear, then move forward again and repeat.
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "include/walker.h"
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"



Walker::Obstacle() {
    for (int i=359; i >= right_ind; i--) {
        if (laserscan_data_range[i] <= this->threshold_dist) {
            return false;
        }
    }
}

Walker::VaccumCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    geometry_msgs::Twist velocity;
    if no obstacle detected {
        velocity.linear.x = 0.5;
        velocity.angular.z = 0.0;
    } else {
        velocity.linear.x = 0.0;
        velocity.angular.z = -1.0;
    }

    vel_pub.publish(velocity);
}

