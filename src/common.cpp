/*
 * ORB-SLAM3 ROS2 Node Implementation
 * Modified for Clearpath Jackal simulation with proper coordinate transforms
 */

#include "ros2_orb_slam3/common.hpp"

MonocularMode::MonocularMode() : Node("mono_node_cpp")
{
    homeDir = getenv("HOME");
    
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");
    this->declare_parameter("node_name_arg", "not_given");
    this->declare_parameter("voc_file_arg", "file_not_set");
    this->declare_parameter("settings_file_path_arg", "file_path_not_set");
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("camera_frame", "camera_link");
    
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";
    
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();
    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();
    
    world_frame_id_ = this->get_parameter("world_frame").as_string();
    camera_frame_id_ = this->get_parameter("camera_frame").as_string();
  
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }
    
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings";
    pubconfigackName = "/mono_py_driver/exp_settings_ack";
    subImgMsgName = "/mono_py_driver/img_msg";
    subTimestepMsgName = "/mono_py_driver/timestep_msg";
    
    // Subscribers
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
        subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));
    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));
    
    // Publishers
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/camera_pose", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/orb_slam3/trajectory", 10);
    
    // TF Broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Initialize path message
    path_msg_.header.frame_id = world_frame_id_;
    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

MonocularMode::~MonocularMode()
{   
    if (pAgent != nullptr) {
        pAgent->Shutdown();
        pAgent->SaveKeyFrameTrajectoryTUM("/tmp/orb_slam3_trajectory.txt");
        pAgent->SaveTrajectoryTUM("/tmp/orb_slam3_camera_trajectory.txt");
        RCLCPP_INFO(this->get_logger(), "Trajectories saved to /tmp/");
    }
    pass;
}

void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg)
{
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());
    
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout << "Sent response: " << message.data.c_str() << std::endl;
    configAck_publisher_->publish(message);
    
    initializeVSLAM(experimentConfig);
}

void MonocularMode::initializeVSLAM(std::string& configString)
{
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml");
    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true;
    enableOpenCVWindow = true;
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl;
}

void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg)
{
    timeStep = time_msg.data;
}

void MonocularMode::publishPose(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp)
{
    Eigen::Matrix4f Tcw_mat = Tcw.matrix();
    if (Tcw_mat.isZero(1e-6)) {
        return; // Invalid pose, skip
    }
    
    // Convert to world frame (inverse of camera-to-world transform)
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Matrix4f Twc_mat = Twc.matrix();
    
    // Extract rotation and translation in camera frame
    Eigen::Matrix3f R_cam = Twc_mat.block<3,3>(0,0);
    Eigen::Vector3f t_cam = Twc_mat.block<3,1>(0,3);
    
    // Transform from ORB-SLAM3 camera frame (Z-forward, X-right, Y-down) 
    // to ROS frame (X-forward, Y-left, Z-up)
    // Rotation matrix to convert coordinate systems
    Eigen::Matrix3f T_ros_cam;
    T_ros_cam <<  0,  0,  1,
                 -1,  0,  0,
                  0, -1,  0;
    
    Eigen::Matrix3f R_ros = T_ros_cam * R_cam * T_ros_cam.transpose();
    Eigen::Vector3f t_ros = T_ros_cam * t_cam;
    
    Eigen::Quaternionf q(R_ros);
    q.normalize();
    
    // Create PoseStamped message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = world_frame_id_;
    
    pose_msg.pose.position.x = t_ros(0);
    pose_msg.pose.position.y = t_ros(1);
    pose_msg.pose.position.z = t_ros(2);
    
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    
    // Publish pose
    pose_publisher_->publish(pose_msg);
    
    // Add to path and publish
    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(pose_msg);
    path_publisher_->publish(path_msg_);
    
    // Broadcast TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = world_frame_id_;
    tf_msg.child_frame_id = camera_frame_id_;
    
    tf_msg.transform.translation.x = t_ros(0);
    tf_msg.transform.translation.y = t_ros(1);
    tf_msg.transform.translation.z = t_ros(2);
    
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(tf_msg);
}

void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading image");
        return;
    }
    
    // Perform ORB-SLAM3 tracking
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep);
    
    // Publish pose, trajectory, and TF
    rclcpp::Time stamp = this->get_clock()->now();
    publishPose(Tcw, stamp);
}
