// Include file 
#ifndef COMMON_HPP
#define COMMON_HPP

// C++ includes
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <cstring>
#include <sstream>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;

// Include Eigen
#include <Eigen/Dense>

// Include cv-bridge
#include <cv_bridge/cv_bridge.h>

// Include OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.h>

// ORB SLAM 3 includes
#include "System.h"

// Global defs
#define pass (void)0

// Node specific definitions
class MonocularMode : public rclcpp::Node
{   
    public:
        std::string experimentConfig = "";
        double timeStep;
        std::string receivedConfig = "";

        MonocularMode();
        ~MonocularMode();
        
    private:
        // Class internal variables
        std::string homeDir = "";
        std::string packagePath = "ros2_test/src/ros2_orb_slam3/";
        std::string OPENCV_WINDOW = "";
        std::string nodeName = "";
        std::string vocFilePath = "";
        std::string settingsFilePath = "";
        bool bSettingsFromPython = false;
        
        std::string subexperimentconfigName = "";
        std::string pubconfigackName = "";
        std::string subImgMsgName = "";
        std::string subTimestepMsgName = "";

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

        // TF Broadcaster
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // Path message to accumulate trajectory
        nav_msgs::msg::Path path_msg_;

        // Frame IDs
        std::string world_frame_id_ = "map";
        std::string camera_frame_id_ = "camera_link";

        // ORB_SLAM3 related variables
        ORB_SLAM3::System* pAgent;
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false;
        bool enableOpenCVWindow = false;

        // ROS callbacks
        void experimentSetting_callback(const std_msgs::msg::String& msg);
        void Timestep_callback(const std_msgs::msg::Float64& time_msg);
        void Img_callback(const sensor_msgs::msg::Image& msg);
        
        // Helper functions
        void initializeVSLAM(std::string& configString);
        void publishPose(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp);
};

#endif
