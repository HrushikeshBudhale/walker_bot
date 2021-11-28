#pragma once

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <sstream>
#include "ros/ros.h"

class WalkerBot {
 public:
    explicit WalkerBot(ros::NodeHandle*);
    void move_forward(double speed);
    void turn(double speed);
    void stop(void);
    void move_around(void);

 private:
    void scan_callback(const sensor_msgs::LaserScanConstPtr& msg);
    double near = 0.5;
    std::vector<double> distances;
    ros::Publisher pub_vels;
    ros::Subscriber sub_scan;
};
