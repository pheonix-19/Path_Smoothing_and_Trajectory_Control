import rclpy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def make_line_strip(ns, frame, pts, rgba=(0.2,0.8,1.0,1.0), scale=0.03, mid=0):
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = scale
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.pose.orientation.w = 1.0
    for x,y in pts:
        p = Point(); p.x = float(x); p.y = float(y); p.z = 0.01
        m.points.append(p)
    return m

def make_spheres(ns, frame, pts, rgba=(1.0,0.2,0.2,0.9), scale=0.06, start_id=1000):
    ma = MarkerArray()
    for i,(x,y) in enumerate(pts):
        m = Marker()
        m.header.frame_id = frame
        m.ns = ns
        m.id = start_id + i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.02
        m.pose.orientation.w = 1.0
        ma.markers.append(m)
    return ma
