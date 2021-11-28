#include <ros/ros.h>

int main(int argc, char *argv[]) {
    // Initialize the node
    ros::init(argc, argv, "walker_bot_node");
    ros::spin();
    return 0;
}
