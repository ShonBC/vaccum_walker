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
#include <vector>
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
        obstacle_thresh{1.0}  // Meters
        {
        }

    /**
     * @brief Destroy the walker object
     * 
     */
    ~Walker();

    /**
     * @brief Publish to /cmd_vel and subscribe to /scan
     * 
     * @param n ROS Node Handle
     */
    void Vaccum(ros::NodeHandle n);

    /**
     * @brief Check Lidar data if an obstacle is detected within a threshold distance
     * 
     * @param lidar_data Lidar scan data
     * @return true If obstacle is detected
     * @return false If no obstacle is detected
     */
    bool Obstacle(const std::vector<float>& lidar_data);

    /**
     * @brief Callback Function to determine velocities to publish
     * 
     * @param scan_msg Lidar scan info from /scan topic
     */
    void VaccumCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};
