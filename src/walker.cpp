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

#include "../include/walker.h"

Walker::~Walker() {
}

void Walker::Vaccum(ros::NodeHandle n) {
    this->n = n;
    vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 100);

    scan_sub = this->n.subscribe(this->scan_topic, 100,
                                &Walker::VaccumCallBack, this);

    ros::spinOnce();
}

bool Walker::Obstacle(const std::vector<float>& lidar_data) {
    ROS_INFO_STREAM("Checking for obstacle");

    for (int i = 30; i < 150; i++) {
        if (lidar_data[i] <= obstacle_thresh) {
            return true;
        }
    }
    return false;
}

void Walker::VaccumCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    geometry_msgs::Twist velocity;
    std::vector<float> lidar_data = scan_msg->ranges;

    if (Obstacle(lidar_data) != true) {
        ROS_INFO_STREAM("No obstacles detected, driving forward");
        velocity.linear.x = 0.5;
        velocity.angular.z = 0.0;
    } else {
        ROS_INFO_STREAM("Obstacle detected, turning");
        velocity.linear.x = 0.0;
        velocity.angular.z = -0.5;
    }

    vel_pub.publish(velocity);
}

