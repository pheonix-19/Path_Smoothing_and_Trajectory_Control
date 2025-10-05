#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math
import time

def yaw_from_quat(q):
    # q = (x, y, z, w)
    siny_cosp = 2.0 * (q[3]*q[2] + q[0]*q[1])
    cosy_cosp = 1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2])
    return math.atan2(siny_cosp, cosy_cosp)

class DebugController(Node):
    def __init__(self):
        super().__init__('debug_controller')
        
        # Subscribe to all relevant topics
        self.create_subscription(Path, 'raw_waypoints', self.on_raw_waypoints, 10)
        self.create_subscription(Path, 'smoothed_path', self.on_smoothed_path, 10)
        self.create_subscription(Path, 'timed_trajectory', self.on_trajectory, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        
        # Create publishers to monitor communication
        self.cmd_echo_pub = self.create_publisher(Twist, '/cmd_vel_echo', 10)
        self.debug_pose_pub = self.create_publisher(PoseStamped, '/robot_debug_pose', 10)
        
        # Create a timer for periodic status checks
        self.timer = self.create_timer(1.0, self.status_check)
        
        # Initialize state variables
        self._raw_waypoints = None
        self._smoothed_path = None
        self._trajectory = None
        self._pose = None
        self._cmd_vel = None
        self._last_received = {
            'raw_waypoints': None,
            'smoothed_path': None,
            'trajectory': None,
            'odom': None,
            'cmd_vel': None
        }
        
        self.get_logger().info('Debug Controller started. Monitoring navigation pipeline...')

    def on_raw_waypoints(self, msg):
        self._raw_waypoints = msg
        self._last_received['raw_waypoints'] = time.time()
        count = len(msg.poses)
        self.get_logger().info(f"RECEIVED raw_waypoints with {count} points")
    
    def on_smoothed_path(self, msg):
        self._smoothed_path = msg
        self._last_received['smoothed_path'] = time.time()
        count = len(msg.poses)
        self.get_logger().info(f"RECEIVED smoothed_path with {count} points")
    
    def on_trajectory(self, msg):
        self._trajectory = msg
        self._last_received['trajectory'] = time.time()
        count = len(msg.poses)
        self.get_logger().info(f"RECEIVED timed_trajectory with {count} points")
    
    def on_odom(self, msg):
        self._pose = msg
        self._last_received['odom'] = time.time()
        
        # Publish debug pose for visualization
        debug_pose = PoseStamped()
        debug_pose.header = msg.header
        debug_pose.pose = msg.pose.pose
        self.debug_pose_pub.publish(debug_pose)
    
    def on_cmd_vel(self, msg):
        self._cmd_vel = msg
        self._last_received['cmd_vel'] = time.time()
        
        # Echo back cmd_vel for debugging
        self.cmd_echo_pub.publish(msg)
        
        # Log velocity commands
        self.get_logger().info(f"CMD_VEL: linear={msg.linear.x:.3f} m/s, angular={msg.angular.z:.3f} rad/s")
    
    def status_check(self):
        now = time.time()
        self.get_logger().info("=== NAVIGATION PIPELINE STATUS ===")
        
        # Check if topics are being published
        for topic, last_time in self._last_received.items():
            if last_time is None:
                self.get_logger().error(f"❌ No messages received on {topic}")
            else:
                delay = now - last_time
                if delay > 5.0:
                    self.get_logger().warn(f"⚠️  {topic}: Last message {delay:.1f}s ago")
                else:
                    self.get_logger().info(f"✅ {topic}: Active")
        
        # Check if we have trajectory but no commands
        if (self._trajectory is not None and 
            self._pose is not None and 
            (self._cmd_vel is None or now - self._last_received['cmd_vel'] > 1.0)):
            self.get_logger().error("❌ CRITICAL: Have trajectory and pose but no cmd_vel being published!")
        
        # Check if we're at the goal but still trying to move
        if self._trajectory is not None and self._pose is not None and self._cmd_vel is not None:
            if len(self._trajectory.poses) > 0:
                end_point = self._trajectory.poses[-1].pose.position
                robot_point = self._pose.pose.pose.position
                dist = math.sqrt((end_point.x - robot_point.x)**2 + (end_point.y - robot_point.y)**2)
                self.get_logger().info(f"Distance to goal: {dist:.3f} m")
        
        # Calculate lookahead point if we have data
        if self._trajectory is not None and self._pose is not None:
            self.calculate_lookahead_debug()
    
    def calculate_lookahead_debug(self):
        """Recreate controller calculation for debugging"""
        try:
            if len(self._trajectory.poses) == 0:
                return
            
            # Extract robot pose
            p = self._pose.pose.pose.position
            o = self._pose.pose.pose.orientation
            yaw = yaw_from_quat((o.x, o.y, o.z, o.w))
            robot_pose = np.array([p.x, p.y, yaw], dtype=float)
            
            # Extract trajectory
            traj = np.array([[p.pose.position.x, p.pose.position.y] 
                             for p in self._trajectory.poses], dtype=float)
            
            # Find closest point
            dists = np.linalg.norm(traj - robot_pose[:2], axis=1)
            closest_idx = np.argmin(dists)
            closest_point = traj[closest_idx]
            
            # Try to find lookahead (0.5m forward)
            Ld = 0.5  # lookahead distance
            for i in range(closest_idx, len(traj)):
                dist_to_point = np.linalg.norm(traj[i] - robot_pose[:2])
                if dist_to_point >= Ld:
                    target = traj[i]
                    self.get_logger().info(f"Lookahead target: ({target[0]:.2f}, {target[1]:.2f}) - {dist_to_point:.2f}m away")
                    
                    # Transform to robot frame
                    dx = target[0] - robot_pose[0]
                    dy = target[1] - robot_pose[1]
                    th = robot_pose[2]
                    x_r = math.cos(-th)*dx - math.sin(-th)*dy
                    y_r = math.sin(-th)*dx + math.cos(-th)*dy
                    
                    # Calculate curvature
                    L_actual = math.sqrt(x_r*x_r + y_r*y_r)
                    curvature = 0.0 if L_actual < 1e-6 else (2.0 * y_r) / (L_actual*L_actual)
                    v_nom = 0.18
                    
                    # Calculate expected angular velocity
                    w = curvature * v_nom
                    
                    self.get_logger().info(f"EXPECTED CONTROLS: v={v_nom:.3f} m/s, w={w:.3f} rad/s")
                    self.get_logger().info(f"CURVATURE: {curvature:.3f}, y_r={y_r:.3f}, L={L_actual:.3f}")
                    
                    # Compare with actual command if available
                    if self._cmd_vel is not None:
                        v_actual = self._cmd_vel.linear.x
                        w_actual = self._cmd_vel.angular.z
                        self.get_logger().info(f"ACTUAL CONTROLS: v={v_actual:.3f} m/s, w={w_actual:.3f} rad/s")
                        
                        # Calculate difference
                        v_diff = v_actual - v_nom
                        w_diff = w_actual - w
                        if abs(w_diff) > 0.2:
                            self.get_logger().warn(f"⚠️  Large angular velocity difference: {w_diff:.3f} rad/s")
                    
                    return
            
            self.get_logger().warn("Could not find lookahead point beyond closest point")
        except Exception as e:
            self.get_logger().error(f"Error in debug calculation: {e}")

def main():
    rclpy.init()
    node = DebugController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()