#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from datetime import datetime

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        
        # Parameters
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to cmd_vel
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        
        # Timer for publishing
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.timer = self.create_timer(1.0/rate, self.publish_odom)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.omega = 0.0
        
        # Last update time
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Fake odometry publisher started')
    
    def on_cmd_vel(self, msg):
        # Update velocities from cmd_vel
        self.v_x = msg.linear.x
        self.omega = msg.angular.z
    
    def publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = now
        
        # Update position based on velocity
        if abs(self.omega) < 0.0001:
            # Straight line motion
            self.x += self.v_x * dt * math.cos(self.theta)
            self.y += self.v_x * dt * math.sin(self.theta)
        else:
            # Arc motion
            v_over_omega = self.v_x / self.omega
            self.x += -v_over_omega * math.sin(self.theta) + v_over_omega * math.sin(self.theta + self.omega * dt)
            self.y += v_over_omega * math.cos(self.theta) - v_over_omega * math.cos(self.theta + self.omega * dt)
            self.theta += self.omega * dt
        
        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create quaternion from yaw
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.theta / 2)
        qw = math.cos(self.theta / 2)
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        odom.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Set velocity
        odom.twist.twist.linear.x = self.v_x
        odom.twist.twist.angular.z = self.omega
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Publish transform
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    node = FakeOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()