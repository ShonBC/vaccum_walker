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

#pragma once
#include <string>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class Walker {
 public:
    ros::NodeHandle n;
    ros::Publisher vel_pub;
    std::string cmd_vel_topic;
    ros::Subscriber scan_sub;
    std::string scan_topic;
    float obstacle_thresh;

    Walker() :
        cmd_vel_topic{"/cmd_vel"},
        scan_topic{"/scan"},
        obstacle_thresh{0.5}  // Meters
        {
        }

    /**
     * @brief Destroy the walker object
     * 
     */
    ~Walker();

    /**
     * @brief Subscribe to /scan to see when an obstacle is detected
     * 
     */
    void Vaccum();

    void VaccumCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};
