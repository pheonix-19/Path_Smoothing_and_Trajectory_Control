import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from nav_pipeline.viz_utils import make_line_strip
from nav_pipeline.utils import resample_by_arclength


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('a_max', 0.5)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('frame_id', 'odom')

        self.sub = self.create_subscription(Path, 'smoothed_path', self.on_path, 10)
        self.pub = self.create_publisher(Path, 'timed_trajectory', 10)
        self.mpub = self.create_publisher(MarkerArray, 'trajectory_markers', 10)

        self.get_logger().info('TrajectoryGenerator ready.')

    # --------------------------------------------------------------
    def on_path(self, msg: Path):
        """Receive smoothed path → build and publish time-parameterized trajectory."""
        if not msg.poses:
            self.get_logger().warn('Empty smoothed path.')
            return

        # --- extract and clean input points ---
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        keep = np.concatenate([[True], diffs > 1e-6])
        pts = pts[keep]
        if pts.shape[0] < 2:
            self.get_logger().warn('Too few valid points for trajectory.')
            return

        # --- compute total arc length ---
        seglen = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seglen)])
        total_len = s[-1]
        if total_len < 1e-6:
            self.get_logger().warn('Zero-length path.')
            return

        # --- parameters ---
        v_max = float(self.get_parameter('v_max').value)
        a_max = float(self.get_parameter('a_max').value)
        ds = 0.05

        # --- resample path uniformly ---
        pts_res = resample_by_arclength(pts, ds=ds)
        diffs2 = np.linalg.norm(np.diff(pts_res, axis=0), axis=1)
        keep2 = np.concatenate([[True], diffs2 > 1e-8])
        pts_res = pts_res[keep2]

        # --- trapezoidal velocity profile ---
        T_accel = v_max / a_max
        S_accel = 0.5 * a_max * T_accel**2
        if 2 * S_accel > total_len:
            S_accel = total_len / 2
            T_accel = np.sqrt(2 * S_accel / a_max)

        S_cruise = total_len - 2 * S_accel
        T_cruise = S_cruise / v_max
        total_time = 2 * T_accel + T_cruise

        s_new = np.arange(0, total_len + 1e-6, ds)
        t_samples = []
        for s_i in s_new:
            if s_i < S_accel:
                t = np.sqrt(2 * s_i / a_max)
            elif s_i < S_accel + S_cruise:
                t = T_accel + (s_i - S_accel) / v_max
            else:
                s_rem = total_len - s_i
                t = total_time - np.sqrt(2 * s_rem / a_max)
            t_samples.append(t)
        t_samples = np.array(t_samples)

        # --- build timed Path ---
        out = Path()
        out.header.frame_id = msg.header.frame_id or self.get_parameter('frame_id').value
        out.header.stamp = self.get_clock().now().to_msg()

        for (x, y), t in zip(pts_res, t_samples[: len(pts_res)]):
            ps = PoseStamped()
            ps.header.frame_id = out.header.frame_id
            ps.header.stamp = out.header.stamp
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            ps.pose.orientation.z = float(t)
            out.poses.append(ps)

        # --- final cleaning: remove duplicates / extra zeros / NaNs ---
        cleaned = []
        for ps in out.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            if not np.isfinite(x) or not np.isfinite(y):
                continue
            if len(cleaned) > 0:
                last = cleaned[-1].pose.position
                if abs(x - last.x) < 1e-6 and abs(y - last.y) < 1e-6:
                    continue
            if len(cleaned) > 0 and abs(x) < 1e-6 and abs(y) < 1e-6:
                continue
            cleaned.append(ps)
        out.poses = cleaned

        if len(out.poses) < 2:
            self.get_logger().warn('Cleaned trajectory too short.')
            return

        # --- publish final Path ---
        self.pub.publish(out)

        # --- RViz marker visualization ---
        line_pts = np.array([[p.pose.position.x, p.pose.position.y] for p in out.poses])
        line = make_line_strip('trajectory', out.header.frame_id, line_pts,
                               rgba=(0.1, 0.3, 1.0, 0.9), scale=0.03, mid=10)
        marr = MarkerArray()
        marr.markers.append(line)
        self.mpub.publish(marr)

        self.get_logger().info(
            f"Published cleaned timed trajectory: {len(out.poses)} pts, "
            f"L={total_len:.2f} m, T={total_time:.2f} s."
        )


# --------------------------------------------------------------
def main():
    rclpy.init()
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
