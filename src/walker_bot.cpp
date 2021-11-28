/**
 * @file walker_bot.cpp
 * @author Hrushikesh Budhale (hbudhale@umd.edu)
 * @brief Library for WalkerBot class
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

#include <walker_bot/walker_bot.hpp>

WalkerBot::WalkerBot(ros::NodeHandle* nh) {
    // create vector for storing 3 distances
    distances.push_back(0);
    distances.push_back(0);
    distances.push_back(0);

    // Advertise topic to publish
    pub_vels = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 5);

    // Create subscriber callback for laser scan data
    sub_scan = nh->subscribe("/scan", 10, &WalkerBot::scan_callback, this);
}

void WalkerBot::scan_callback(const sensor_msgs::LaserScanConstPtr& msg) {
    distances[0] = msg->ranges[30];
    distances[1] = msg->ranges[0];
    distances[2] = msg->ranges[330];
    ROS_INFO_STREAM("[Walker_bot] " << distances[0] << "\t"
                                    << distances[1] << "\t"
                                    << distances[2] << "\n");
    move_around();
}

void WalkerBot::move_forward(double speed = 0.26) {
    geometry_msgs::Twist new_msg;
    new_msg.linear.x = speed;
    pub_vels.publish(new_msg);
}

void WalkerBot::turn(double dir) {
    geometry_msgs::Twist new_msg;
    new_msg.linear.x = -0.2;
    new_msg.angular.z = dir*1.82;
    pub_vels.publish(new_msg);
}

void WalkerBot::stop(void) {
    geometry_msgs::Twist new_msg;
    new_msg.linear.x = 0;
    new_msg.angular.z = 0;
    pub_vels.publish(new_msg);
}

void WalkerBot::move_around(void) {
    // if front is near obstacle
    if (distances[1] < near) {
        ROS_INFO_STREAM("[Walker_bot] Taking turn");
        // compare left and right
        if (distances[0] > distances[2]) {
            // left obstacle is farther
            turn(1);  // left
        } else {
            turn(-1);  // right
        }
    } else {
        ROS_INFO_STREAM("[Walker_bot] Moving forward");
        move_forward();
    }
}
