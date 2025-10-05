import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
import yaml
import os

class WaypointServer(Node):
    def __init__(self):
        super().__init__('waypoint_server')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('frame_id', 'map')

        self.path_pub = self.create_publisher(Path, 'raw_waypoints', 10)

        self.timer = self.create_timer(1.0, self.publish_waypoints)
        self.add_on_set_parameters_callback(self.on_params)

        self._path_msg = None
        self.load_waypoints()

    def on_params(self, params):
        for p in params:
            if p.name == 'waypoint_file':
                self.get_logger().info('Reloading waypoints...')
                self.load_waypoints()
        return SetParametersResult(successful=True)

    def load_waypoints(self):
        fpath = self.get_parameter('waypoint_file').get_parameter_value().string_value
        frame = self.get_parameter('frame_id').get_parameter_value().string_value
        if not fpath:
            self.get_logger().warn('No waypoint_file set.')
            return
        if not os.path.exists(fpath):
            self.get_logger().error(f'Waypoint file not found: {fpath}')
            return

        with open(fpath, 'r') as f:
            data = yaml.safe_load(f)

        waypoints = data.get('waypoints', [])
        path = Path()
        path.header.frame_id = frame
        for x,y in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_msg = path
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints.')

    def publish_waypoints(self):
        if self._path_msg:
            self._path_msg.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self._path_msg)

def main():
    rclpy.init()
    node = WaypointServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
