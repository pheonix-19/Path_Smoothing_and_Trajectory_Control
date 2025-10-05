import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from nav_pipeline.viz_utils import make_line_strip, make_spheres
from nav_pipeline.utils import resample_by_arclength

def catmull_rom_spline(points, samples_per_seg=10, alpha=0.5, closed=False):
    """
    Safe Catmull-Rom spline interpolation for 2D points.
    Handles duplicate points and prevents divide-by-zero errors.
    """
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 2:
        return pts

    # remove consecutive duplicates / zero-distance points
    diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = np.concatenate([[True], diffs > 1e-6])
    pts = pts[keep]
    if pts.shape[0] < 2:
        return pts

    if closed:
        P = np.vstack([pts[-1], pts, pts[0], pts[1]])
    else:
        P = np.vstack([pts[0], pts, pts[-1], pts[-1]])

    out = []

    def tj(ti, pi, pj):
        d = np.linalg.norm(pj - pi)
        return ti + np.power(d, alpha) if d > 1e-9 else ti + 1e-6

    def safe_div(a, b):
        return a / b if abs(b) > 1e-9 else a / 1e-6

    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)
        t = np.linspace(t1, t2, samples_per_seg)

        denom = lambda a, b: (a - b) if abs(a - b) > 1e-9 else 1e-6

        A1 = (t1 - t)[:, None] / denom(t1, t0) * p0 + (t - t0)[:, None] / denom(t1, t0) * p1
        A2 = (t2 - t)[:, None] / denom(t2, t1) * p1 + (t - t1)[:, None] / denom(t2, t1) * p2
        A3 = (t3 - t)[:, None] / denom(t3, t2) * p2 + (t - t2)[:, None] / denom(t3, t2) * p3

        B1 = (t2 - t)[:, None] / denom(t2, t0) * A1 + (t - t0)[:, None] / denom(t2, t0) * A2
        B2 = (t3 - t)[:, None] / denom(t3, t1) * A2 + (t - t1)[:, None] / denom(t3, t1) * A3

        C = (t2 - t)[:, None] / denom(t2, t1) * B1 + (t - t1)[:, None] / denom(t2, t1) * B2
        C = C[np.isfinite(C).all(axis=1)]
        if C.size > 0:
            out.append(C)

    if len(out) == 0:
        return pts
    return np.vstack(out)

class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('samples_per_segment', 15)
        self.declare_parameter('resample_ds', 0.03)
        self.declare_parameter('closed', False)

        self.sub = self.create_subscription(Path, 'raw_waypoints', self.on_waypoints, 10)
        self.pub = self.create_publisher(Path, 'smoothed_path', 10)
        self.mpub = self.create_publisher(MarkerArray, 'smoothed_markers', 10)

        self.get_logger().info('PathSmoother ready.')

    def on_waypoints(self, msg: Path):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(pts) < 2:
            self.get_logger().warn('Not enough waypoints to smooth.')
            return

        sps = int(self.get_parameter('samples_per_segment').value)
        closed = bool(self.get_parameter('closed').value)
        ds = float(self.get_parameter('resample_ds').value)

        smooth = catmull_rom_spline(pts, samples_per_seg=sps, closed=closed)
        if smooth.shape[0] < 2:
            self.get_logger().warn('Spline generation failed.')
            return

        smooth_res = resample_by_arclength(smooth, ds=ds)
        if smooth_res.shape[0] < 2:
            self.get_logger().warn('Resampling failed.')
            return

        out = Path()
        out.header.frame_id = msg.header.frame_id or self.get_parameter('frame_id').value
        out.header.stamp = self.get_clock().now().to_msg()
        for x, y in smooth_res:
            ps = PoseStamped()
            ps.header.frame_id = out.header.frame_id
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)

        self.pub.publish(out)

        # Publish RViz markers
        lm = make_line_strip('smoothed', out.header.frame_id, smooth_res,
                             rgba=(0.2, 0.8, 0.2, 1.0), scale=0.03, mid=1)
        ma = make_spheres('waypoints', out.header.frame_id, pts,
                          rgba=(1.0, 0.3, 0.3, 0.9), scale=0.07, start_id=2000)
        marr = MarkerArray()
        marr.markers.append(lm)
        marr.markers.extend(ma.markers)
        self.mpub.publish(marr)

        self.get_logger().info(f'Published smoothed path with {len(out.poses)} points.')

def main():
    rclpy.init()
    node = PathSmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
