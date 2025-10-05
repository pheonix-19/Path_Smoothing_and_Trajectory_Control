import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_pipeline.utils import cumulative_lengths
import numpy as np

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        # Velocity profile params
        self.declare_parameter('v_max', 0.25)      # m/s
        self.declare_parameter('a_max', 0.5)       # m/s^2
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 5.0)    # Hz (republish latest trajectory)

        self.sub = self.create_subscription(Path, 'smoothed_path', self.on_path, 10)
        self.pub = self.create_publisher(Path, 'timed_trajectory', 10)

        self.timer = self.create_timer(1.0/float(self.get_parameter('publish_rate').value), self.repub)
        self._latest = None

        self.get_logger().info('TrajectoryGenerator ready.')

    def on_path(self, msg: Path):
        if len(msg.poses) < 2:
            return

        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float)
        s = cumulative_lengths(pts)
        L = s[-1]
        v_max = float(self.get_parameter('v_max').value)
        a_max = float(self.get_parameter('a_max').value)

        # Trapezoidal: accelerate, cruise, decelerate
        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc**2

        if 2*d_acc > L:
            # Triangle profile
            v_peak = np.sqrt(a_max * L)
            t_acc = v_peak / a_max
            t_total = 2 * t_acc
            def time_at_s(si):
                if si <= 0.5*L:
                    return np.sqrt(2*si/a_max)
                else:
                    sr = L - si
                    return t_total - np.sqrt(2*sr/a_max)
        else:
            d_cruise = L - 2*d_acc
            t_cruise = d_cruise / v_max
            t_total = 2*t_acc + t_cruise
            def time_at_s(si):
                if si < d_acc:
                    return np.sqrt(2*si/a_max)
                elif si < d_acc + d_cruise:
                    return t_acc + (si - d_acc)/v_max
                else:
                    sd = L - si
                    return t_acc + t_cruise + (t_acc - np.sqrt(2*sd/a_max))

        t_list = [time_at_s(si) for si in s]

        out = Path()
        out.header = msg.header
        for (x,y), t in zip(pts, t_list):
            ps = PoseStamped()
            ps.header.frame_id = out.header.frame_id
            ps.header.stamp = msg.header.stamp
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            # encode time in pose.orientation.z for quick visibility (non-standard, but handy in tests)
            ps.pose.orientation.z = float(t)
            out.poses.append(ps)

        self._latest = out
        self.pub.publish(out)
        self.get_logger().info(f'Generated timed trajectory: {len(out.poses)} points, total length {L:.2f} m, est T ~ {t_total:.2f} s')

    def repub(self):
        if self._latest:
            self._latest.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self._latest)

def main():
    rclpy.init()
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
