#!/usr/bin/env python3
"""
Build a point cloud map using ORB-SLAM3 poses and LiDAR data
Subscribes to camera poses from ORB-SLAM3 and LiDAR point clouds,
transforms LiDAR points to global frame and accumulates them.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header


class PointCloudMapper(Node):
    def __init__(self):
        super().__init__('pointcloud_mapper')
        
        # Parameters
        self.declare_parameter('lidar_topic', '/j100_0000/sensors/lidar3d_0/points')
        self.declare_parameter('pose_topic', '/orb_slam3/camera_pose')
        self.declare_parameter('map_topic', '/point_cloud_map')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_points', 5000000)
        self.declare_parameter('points_per_scan', 1000)
        
        lidar_topic = self.get_parameter('lidar_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        map_topic = self.get_parameter('map_topic').value
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self.lidar_callback, 10)
        
        # Publisher
        self.map_pub = self.create_publisher(PointCloud2, map_topic, 10)
        
        # State
        self.current_pose = None
        self.map_points = []
        self.frame_count = 0
        self.publish_interval = 10
        
        self.get_logger().info(f"PointCloud Mapper initialized")
        self.get_logger().info(f"  LiDAR topic: {lidar_topic}")
        self.get_logger().info(f"  Pose topic: {pose_topic}")
        self.get_logger().info(f"  Map topic: {map_topic}")

    def pose_callback(self, msg):
        self.current_pose = msg

    def lidar_callback(self, msg):
        if self.current_pose is None:
            return
        
        self.frame_count += 1
        
        # Extract pose
        pos = self.current_pose.pose.position
        ori = self.current_pose.pose.orientation
        
        # Create transformation
        rotation = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        translation = np.array([pos.x, pos.y, pos.z])
        
        # Read points from PointCloud2
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        
        if len(points) == 0:
            return
        
        points = np.array(points)
        
        # Transform points to global frame
        transformed = rotation.apply(points) + translation
        
        # Subsample to reduce memory
        points_per_scan = self.get_parameter('points_per_scan').value
        step = max(1, len(transformed) // points_per_scan)
        self.map_points.extend(transformed[::step].tolist())
        
        # Limit total points
        max_points = self.get_parameter('max_points').value
        if len(self.map_points) > max_points:
            self.map_points = self.map_points[-max_points:]
        
        # Publish map periodically
        if self.frame_count % self.publish_interval == 0:
            self.publish_map()
            self.get_logger().info(f"Map has {len(self.map_points)} points")

    def publish_map(self):
        if len(self.map_points) == 0:
            return
        
        points = np.array(self.map_points, dtype=np.float32)
        
        # Create header
        map_header = Header()
        map_header.stamp = self.get_clock().now().to_msg()
        map_header.frame_id = "map"
        
        # Create PointCloud2
        cloud_msg = pc2.create_cloud_xyz32(map_header, points)
        self.map_pub.publish(cloud_msg)

    def save_map(self, filename='/tmp/point_cloud_map.xyz'):
        if len(self.map_points) > 0:
            points = np.array(self.map_points)
            np.savetxt(filename, points, fmt='%.6f')
            self.get_logger().info(f"Map saved to {filename} with {len(points)} points")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Save map on exit
    node.save_map()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
