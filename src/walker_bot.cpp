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
