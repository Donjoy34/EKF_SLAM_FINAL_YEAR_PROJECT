from eufs_msgs.msg import WaypointArrayStamped, Waypoint, ConeArrayWithCovariance
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker
import numpy as np
import math

class AccelerationPlanner(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Acceleration node started")
        self.create_subscription(ConeArrayWithCovariance, "/cones", self.on_cones, 1)
        self.pub_track_line = self.create_publisher(WaypointArrayStamped, "/trajectory", 1)
    
    def on_cones(self, msg):
        for cone in msg.big_orange_cones:
            if(cone.point.y < 0):
                msg.yellow_cones.append(cone)
            else:
                msg.blue_cones.append(cone)
        
        midpoints = self.calculate_midpoints(
            msg.yellow_cones, 
            msg.blue_cones, 
            "safe"
        )

        midpoints += self.calculate_midpoints(
            [c for c in msg.orange_cones if c.point.y > 0], 
            [c for c in msg.orange_cones if c.point.y < 0], 
            "danger"
        )

        midpoints = self.order_cones_forward(midpoints, filter_radius=3)
        midpoints.insert(0, [0.0, 0.0, "danger"])
        midpoints.insert(1, [3.0, 0.0, "safe"])
        midpoints = self.to_bezier(midpoints, 0.5)

        try:
            self.publish_path(midpoints)
        except:
            self.get_logger().info("Error Publishing Path.")
            pass


    def find_closest_cone(self, p, cones, render_distance=99):
        if not p or not cones:
            return None
        
        closest_cone = None
        min_distance = float('inf')
        
        for cone in cones:
            cx, cy = cone.point.x - p.point.x, cone.point.y - p.point.y
            cone_distance = math.sqrt(cx**2 + cy**2)
            
            if cone_distance > render_distance:
                continue
            
            if cone_distance < min_distance:
                min_distance = cone_distance
                closest_cone = cone
        
        return closest_cone
    
    def calculate_midpoints(self, c1, c2, mp_type):
        arr = []

        for i in range(min(len(c1), len(c2))):
            cc = self.find_closest_cone(c2[i], c1, 5)
            if(not cc): continue
            xc = (c2[i].point.x + cc.point.x) / 2
            yc = ((c2[i].point.y + cc.point.y) / 2)
            mp = [xc, yc, mp_type]
                    
            arr.append(mp)

        return arr
    
    def publish_path(self, midpoints):
        waypoint_array = WaypointArrayStamped()
        waypoint_array.header.frame_id = "base_footprint"
        waypoint_array.header.stamp = self.get_clock().now().to_msg()

        for p in midpoints:
            point = Point(x=p[0], y=p[1])
            waypoint = Waypoint(position=point, type=p[2])
            waypoint_array.waypoints.append(waypoint)

        self.pub_track_line.publish(waypoint_array)


    def order_cones_forward(self, cones, filter_radius):
        in_range = [
            c for c in cones
            if math.hypot(c[0], c[1]) > filter_radius
        ]
        return sorted(in_range, key=lambda c: c[0])


    def to_bezier(self, control_pts, spacing: float = 0.5):
        if len(control_pts) < 2:
            return control_pts
        coords  = [pt[:2] for pt in control_pts]
        labels  = [pt[2]  for pt in control_pts]
        def bezier(t, pts_xy):
            n = len(pts_xy) - 1
            return sum(
                math.comb(n, i) * (1 - t) ** (n - i) * t ** i * np.array(pts_xy[i])
                for i in range(n + 1)
            )

        def dense_curve(pts_xy, steps=1000):
            ts       = np.linspace(0.0, 1.0, steps)
            samples  = np.array([bezier(t, pts_xy) for t in ts])
            seg_lens = np.linalg.norm(np.diff(samples, axis=0), axis=1)
            cum_dist = np.insert(np.cumsum(seg_lens), 0, 0.0)
            return samples, cum_dist

        curve_xy, cum_dist = dense_curve(coords)
        total_len = cum_dist[-1]
        targets   = np.arange(0.0, total_len, spacing)
        resampled_xy = []
        idx = 0
        for s in targets:
            while idx < len(cum_dist) - 1 and cum_dist[idx] < s:
                idx += 1
            if idx == 0:
                resampled_xy.append(curve_xy[0])
            else:
                t = (s - cum_dist[idx - 1]) / (cum_dist[idx] - cum_dist[idx - 1])
                p = curve_xy[idx - 1] + t * (curve_xy[idx] - curve_xy[idx - 1])
                resampled_xy.append(p)
        resampled_with_type = []
        origin_xy = np.array(coords)
        for p in resampled_xy:
            nearest = np.argmin(np.linalg.norm(origin_xy - p, axis=1))
            resampled_with_type.append([float(p[0]), float(p[1]), labels[nearest]])

        return resampled_with_type
    