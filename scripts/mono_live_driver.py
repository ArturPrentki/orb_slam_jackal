#!/usr/bin/env python3
"""
Live camera driver for ORB-SLAM3 Monocular mode
Subscribes to a ROS2 camera topic and sends images to the ORB-SLAM3 C++ node
"""

import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError


class MonoLiveDriver(Node):
    def __init__(self, node_name="mono_live_driver"):
        super().__init__(node_name)
        
        # Declare parameters
        self.declare_parameter("settings_name", "Jackal_Sim")
        self.declare_parameter("camera_topic", "/j100_0000/sensors/camera_0/color/image")
        
        # Get parameters
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.camera_topic = str(self.get_parameter('camera_topic').value)
        
        self.get_logger().info(f"Settings: {self.settings_name}")
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        
        # CV Bridge
        self.br = CvBridge()
        
        # Handshake flag
        self.send_config = True
        self.frame_id = 0
        
        # Publishers for ORB-SLAM3 C++ node
        self.pub_exp_config = self.create_publisher(
            String, "/mono_py_driver/experiment_settings", 1)
        self.pub_img = self.create_publisher(
            Image, "/mono_py_driver/img_msg", 1)
        self.pub_timestep = self.create_publisher(
            Float64, "/mono_py_driver/timestep_msg", 1)
        
        # Subscriber for acknowledgment
        self.sub_ack = self.create_subscription(
            String, "/mono_py_driver/exp_settings_ack", 
            self.ack_callback, 10)
        
        # Camera subscriber (created after handshake)
        self.camera_sub = None
        
        self.get_logger().info("MonoLiveDriver initialized, waiting for handshake...")

    def ack_callback(self, msg):
        if msg.data == "ACK":
            self.send_config = False
            self.get_logger().info("Handshake complete!")
            
            # Now subscribe to camera
            self.camera_sub = self.create_subscription(
                Image, self.camera_topic,
                self.image_callback, 10)
            self.get_logger().info(f"Subscribed to {self.camera_topic}")

    def handshake(self):
        if self.send_config:
            msg = String()
            msg.data = self.settings_name
            self.pub_exp_config.publish(msg)
            time.sleep(0.01)

    def image_callback(self, img_msg):
        try:
            self.frame_id += 1
            
            # Get timestamp
            timestamp = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
            
            # Publish timestamp first, then image (as expected by C++ node)
            timestep_msg = Float64()
            timestep_msg.data = timestamp
            self.pub_timestep.publish(timestep_msg)
            self.pub_img.publish(img_msg)
            
            if self.frame_id % 30 == 0:
                self.get_logger().info(f"Sent frame {self.frame_id}")
                
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MonoLiveDriver()
    
    # Handshake loop
    while node.send_config:
        node.handshake()
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Main loop
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
