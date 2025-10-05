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

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('topic_cmd_vel').value, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(Path, self.get_parameter('trajectory_topic').value, self.on_traj, 10)

        self.timer = self.create_timer(1.0/float(self.get_parameter('control_rate').value), self.on_timer)

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
        # Find first path point at distance >= Ld
        if path is None or len(path) == 0:
            return None
        dists = np.linalg.norm(path - pos[:2], axis=1)
        idx = np.argmax(dists >= Ld)
        if dists[idx] < Ld and idx == len(path)-1:
            return path[-1]
        return path[idx]

    def on_timer(self):
        if self._traj is None or self._pose is None or self._goal_reached:
            return
        Ld = float(self.get_parameter('lookahead').value)
        goal_tol = float(self.get_parameter('goal_tolerance').value)
        v_nom = float(self.get_parameter('v_nominal').value)
        v_min = float(self.get_parameter('v_min').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)

        # goal check
        if np.linalg.norm(self._traj[-1] - self._pose[:2]) < goal_tol:
            self._goal_reached = True
            self.get_logger().info('Goal reached. Stopping.')
            self.cmd_pub.publish(Twist())
            return

        target = self.find_lookahead(self._pose, self._traj, Ld)
        if target is None:
            return

        # Transform target to robot frame
        dx = target[0] - self._pose[0]
        dy = target[1] - self._pose[1]
        th = self._pose[2]
        x_r =  math.cos(-th)*dx - math.sin(-th)*dy
        y_r =  math.sin(-th)*dx + math.cos(-th)*dy

        # Pure pursuit curvature
        if Ld < 1e-6:
            curvature = 0.0
        else:
            curvature = (2.0 * y_r) / (Ld*Ld)

        v = np.clip(v_nom, v_min, v_max)
        w = np.clip(curvature * v, -w_max, w_max)

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
