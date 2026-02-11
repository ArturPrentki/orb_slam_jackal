/*
 * ORB-SLAM3 ROS2 Monocular Node
 * Main entry point for the SLAM node
 */

#include "ros2_orb_slam3/common.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MonocularMode>(); 
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
