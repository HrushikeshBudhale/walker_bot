/**
 * @file walker_bot.hpp
 * @author Hrushikesh Budhale (hbudhale@umd.edu)
 * @brief File containing WalkerBot class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/*
walker_bot
Copyright © 2021
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

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
