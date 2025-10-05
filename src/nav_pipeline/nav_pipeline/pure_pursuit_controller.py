import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import math

def yaw_from_quat(q):
    # q = (x, y, z, w)
    siny_cosp = 2.0 * (q[3]*q[2] + q[0]*q[1])
    cosy_cosp = 1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2])
    return math.atan2(siny_cosp, cosy_cosp)

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        # Declare parameters with defaults
        self.declare_parameter('lookahead', 0.5)
        self.declare_parameter('v_nominal', 0.22)     # typical TB3 linear
        self.declare_parameter('v_min', 0.08)
        self.declare_parameter('v_max', 0.30)
        self.declare_parameter('w_max', 1.5)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('topic_cmd_vel', 'cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('trajectory_topic', 'timed_trajectory')
        self.declare_parameter('control_rate', 20.0)

        # Get parameter values safely
        topic_cmd_vel = self.get_parameter('topic_cmd_vel').get_parameter_value().string_value or 'cmd_vel'
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value or '/odom'
        traj_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value or 'timed_trajectory'
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value or 20.0

        # Create publishers and subscribers with safe parameter values
        self.cmd_pub = self.create_publisher(Twist, topic_cmd_vel, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_subscription(Path, traj_topic, self.on_traj, 10)

        # Create control timer
        self.timer = self.create_timer(1.0/float(control_rate), self.on_timer)

        self._traj = None
        self._pose = None
        self._goal_reached = False
        self.get_logger().info('PurePursuit controller ready.')

    def on_traj(self, msg: Path):
        self._traj = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float)
        self._goal_reached = False
        self.get_logger().info(f'Received trajectory with {len(self._traj)} points.')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = yaw_from_quat((o.x, o.y, o.z, o.w))
        self._pose = np.array([p.x, p.y, yaw], dtype=float)

    def find_lookahead(self, pos, path, Ld):
        """Find lookahead point on path at distance Ld from current position."""
        # Safety check
        if path is None or len(path) == 0:
            self.get_logger().warn("Empty path in find_lookahead")
            return None
        
        # Find closest point on path
        dists = np.linalg.norm(path - pos[:2], axis=1)
        closest_idx = np.argmin(dists)
        self.get_logger().debug(f"Closest point idx: {closest_idx}/{len(path)}, dist: {dists[closest_idx]:.3f}m")
        
        # If we're very close to the end of the path, just return the last point
        if closest_idx >= len(path) - 3:
            self.get_logger().info(f"Near end of path, using final point: {path[-1]}")
            return path[-1]
        
        # Find intersection of circle (center=pos, radius=Ld) with path segments
        # This approach handles paths with sharp turns better
        for i in range(closest_idx, len(path) - 1):
            # Get segment points
            p1 = path[i]
            p2 = path[i + 1]
            
            # Vector from pos to p1
            v1 = p1 - pos[:2]
            d1 = np.linalg.norm(v1)
            
            # Check if p1 is outside circle and p2 is inside circle
            if d1 >= Ld:
                # Found a point beyond lookahead distance
                return p1
                
            # Vector from p1 to p2
            v2 = p2 - p1
            seg_len = np.linalg.norm(v2)
            if seg_len < 1e-6:  # Skip zero-length segments
                continue
                
            # Normalize v2
            v2_unit = v2 / seg_len
                
            # Project v1 onto v2
            proj = np.dot(v1, v2_unit)
                
            # Find closest point on segment to pos
            if proj < 0:
                closest = p1
            elif proj > seg_len:
                closest = p2
            else:
                closest = p1 + proj * v2_unit
                
            # Distance from pos to closest point on segment
            closest_dist = np.linalg.norm(closest - pos[:2])
                
            # If closest point is within lookahead distance and segment passes through circle
            if closest_dist <= Ld and (d1 <= Ld or np.linalg.norm(p2 - pos[:2]) <= Ld):
                # Find intersection point(s)
                # Parametric equation of segment: p1 + t * v2, t in [0,1]
                # Equation of circle: |p - pos| = Ld
                # Solving for t: |p1 + t*v2 - pos| = Ld
                
                # Using quadratic formula:
                a = np.dot(v2, v2)
                b = 2 * np.dot(v1, v2)
                c = np.dot(v1, v1) - Ld * Ld
                
                discriminant = b*b - 4*a*c
                
                if discriminant < 0:
                    # No intersection (shouldn't happen given previous checks)
                    continue
                    
                # Find the two solutions
                t1 = (-b + np.sqrt(discriminant)) / (2*a)
                t2 = (-b - np.sqrt(discriminant)) / (2*a)
                
                # Choose the solution in [0,1] and further along the path
                valid_t = []
                if 0 <= t1 <= 1:
                    valid_t.append(t1)
                if 0 <= t2 <= 1:
                    valid_t.append(t2)
                    
                if valid_t:
                    # Choose the largest valid t value (furthest along segment)
                    t = max(valid_t)
                    intersection = p1 + t * v2
                    self.get_logger().debug(f"Found intersection at segment {i}, t={t:.3f}")
                    return intersection
                    
        # If no intersection found, use the last point
        self.get_logger().info("No lookahead point found, using last path point")
        return path[-1]

    def on_timer(self):
        """Calculate and publish control commands at fixed rate."""
        if self._traj is None:
            self.get_logger().warn("No trajectory received")
            return
            
        if self._pose is None:
            self.get_logger().warn("No odometry received")
            return
            
        if self._goal_reached:
            return
            
        # Get parameters with safe defaults
        Ld = float(self.get_parameter('lookahead').get_parameter_value().double_value or 0.5)
        goal_tol = float(self.get_parameter('goal_tolerance').get_parameter_value().double_value or 0.12)
        v_nom = float(self.get_parameter('v_nominal').get_parameter_value().double_value or 0.22)
        v_min = float(self.get_parameter('v_min').get_parameter_value().double_value or 0.08) 
        v_max = float(self.get_parameter('v_max').get_parameter_value().double_value or 0.30)
        w_max = float(self.get_parameter('w_max').get_parameter_value().double_value or 1.5)

        # Check if we've reached the goal
        dist_to_goal = np.linalg.norm(self._traj[-1] - self._pose[:2])
        self.get_logger().debug(f"Distance to goal: {dist_to_goal:.3f}m (tolerance: {goal_tol:.3f}m)")
        
        if dist_to_goal < goal_tol:
            self._goal_reached = True
            self.get_logger().info(f'Goal reached at {self._pose[:2]}. Stopping.')
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        # Find appropriate lookahead point
        target = self.find_lookahead(self._pose, self._traj, Ld)
        if target is None:
            self.get_logger().warn("No valid lookahead target found")
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        # Transform target to robot-centered coordinate frame
        dx = target[0] - self._pose[0]
        dy = target[1] - self._pose[1]
        th = self._pose[2]
        
        # Rotation matrix from world to robot frame
        x_r = math.cos(-th)*dx - math.sin(-th)*dy
        y_r = math.sin(-th)*dx + math.cos(-th)*dy
        
        # Debug info
        self.get_logger().debug(f"Target world: ({target[0]:.3f}, {target[1]:.3f})")
        self.get_logger().debug(f"Robot pose: ({self._pose[0]:.3f}, {self._pose[1]:.3f}, {math.degrees(th):.1f}°)")
        self.get_logger().debug(f"Target robot frame: ({x_r:.3f}, {y_r:.3f})")

        # Pure pursuit curvature
        L_actual = math.sqrt(x_r*x_r + y_r*y_r)
        if L_actual < 1e-6:
            curvature = 0.0
        else:
            curvature = (2.0 * y_r) / (L_actual*L_actual)
        
        # Scale velocity based on curvature - slow down for sharp turns
        curvature_factor = max(0.5, min(1.0, 1.0 - 0.5 * abs(curvature)))
        v_adjusted = v_nom * curvature_factor
        v = np.clip(v_adjusted, v_min, v_max)
        
        # Calculate angular velocity from curvature and linear velocity
        w = curvature * v
        
        # Cap angular velocity
        w = np.clip(w, -w_max, w_max)
        
        # Log control values
        self.get_logger().debug(f"L_actual={L_actual:.3f}m, curvature={curvature:.3f}, " +
                                f"v={v:.3f}m/s, w={w:.3f}rad/s")

        # Create and publish command
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
