#!/bin/bash
# ORB-SLAM3 Jackal Startup Script
# This script sets up TF relays and starts all necessary nodes

set -e

echo "=========================================="
echo "  ORB-SLAM3 Jackal Startup Script"
echo "=========================================="

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/clearpath_ws/install/setup.bash 2>/dev/null || source /root/clearpath_ws/install/setup.bash

# Kill any existing processes
echo "Cleaning up old processes..."
pkill -f "static_transform_publisher" 2>/dev/null || true
pkill -f "relay" 2>/dev/null || true
sleep 1

# Start TF relays
echo "Starting TF relays..."
ros2 run topic_tools relay /j100_0000/tf /tf &
ros2 run topic_tools relay /j100_0000/tf_static /tf_static &
sleep 2

# Start static transforms
echo "Starting static transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
ros2 run tf2_ros static_transform_publisher 0 0 0.15 0 0 0 base_link lidar3d_0_laser &
sleep 1

echo ""
echo "=========================================="
echo "  TF Setup Complete!"
echo "=========================================="
echo ""
echo "Now run these commands in separate terminals:"
echo ""
echo "Terminal 1 - ORB-SLAM3 Node:"
echo "  ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp"
echo ""
echo "Terminal 2 - Camera Driver:"
echo "  python3 scripts/mono_live_driver.py --ros-args -p settings_name:=Jackal_Sim -p camera_topic:=/j100_0000/sensors/camera_0/color/image"
echo ""
echo "Terminal 3 - Point Cloud Mapper (optional):"
echo "  python3 scripts/pointcloud_mapper.py"
echo ""
echo "Terminal 4 - RViz:"
echo "  rviz2 -d config/orb_slam3.rviz"
echo ""
echo "Terminal 5 - Teleop:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/j100_0000/cmd_vel"
echo ""
echo "=========================================="

# Keep script running to maintain background processes
echo "Press Ctrl+C to stop TF relays..."
wait
