from turtle import distance
from eufs_msgs.msg import WaypointArrayStamped, Waypoint, ConeArrayWithCovariance, CanState, FullState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rclpy.node import Node
import rclpy
import math
from std_msgs.msg import Int16, Bool
from typing import List, Tuple


# from sensor_msgs.msg import gps

import numpy as np


class Planner(Node):
    def __init__(self, name):
        super().__init__(name)
        self.tmp_id = 0

        # Declare ROS parameters
        self.threshold = self.declare_parameter("threshold", 6.0).value
        self.declare_parameter("bezier", True)
        self.declare_parameter("bezeire", True)
        self.declare_parameter("interpolation", True)

        # Create subscribers
        self.cones_sub = self.create_subscription(ConeArrayWithCovariance, "/cones", self.cones_callback, 1)

        # Create publishers
        self.track_line_pub = self.create_publisher(WaypointArrayStamped, "/trajectory", 1)
        self.pub_blue_cones = self.create_publisher(Marker, "/planner/LineStripBlueCones", 1)
        self.pub_yellow_cones = self.create_publisher(Marker, "/planner/LineStripYellowCones", 1)
        self.pub_midpoints = self.create_publisher(Marker, "/planner/LineStripMidPoints", 1)
        self.pub_pair_lines = self.create_publisher(Marker, "/planner/ConePairLines", 1)
        self.pub_circle_margin = self.create_publisher(Marker, "/planner/CircleMargin", 1)


        self.car_state_sub1 = self.create_subscription(CanState, "/ros_can/state", self.state_callback, 1)
        self.control_sig_pub = self.create_publisher(Int16,"/planner/ConSig", 1)
        self.turn_memory_pub = self.create_publisher(Bool, "/planner/turn_memory_active", 1)




        self.lap_count = 0
        self.is_big_orange_cone = False
        self.is_small_orange_cone = False
        self.blue_cone_count = 0
        self.orange_cone_count = 0
        self.yellow_cone_count = 0

        self.yoffset = 0.0 #TODO Needs to calibarte this number with testing
        self.width = 3 #TODO Needs to calibarte this number with testing (Based on testing from simulateor 4.5)
        self.forward_filter_distance = 12 #TODO Needs to calibarte this number with testing
        self.turn_memory_sign = 1.0




    
    def cones_callback(self, msg):
        self.get_logger().info(f"Lap count: {self.lap_count}")
        blue_cones = self.parse_cones_coords(msg.blue_cones)
        yellow_cones = self.parse_cones_coords(msg.yellow_cones)
        self.orange_cone_count = len(msg.orange_cones)
        print(self.orange_cone_count)



      



        for cone in msg.orange_cones:
            if(cone.point.y < 0):
                msg.yellow_cones.append(cone)
            else:
                msg.blue_cones.append(cone)
        
        midpoints, pair_lines = self.calculate_midpoints(
            msg.yellow_cones, 
            msg.blue_cones
        )

        # midpoints += self.calculate_midpoints(
        #     [c for c in msg.orange_cones if c.point.y > 0], 
        #     [c for c in msg.orange_cones if c.point.y < 0]
        # )


        midpoints = self.order_cones_forward(midpoints, filter_radius=3)
        memory_msg = Bool()
        if midpoints:
            self.update_turn_memory_from_midpoints(midpoints)
            path_points = [[0.0, 0.0], [3.0, 0.0]] + midpoints
            use_bezier = bool(self.get_parameter("bezier").value) and bool(self.get_parameter("bezeire").value)
            use_interp = bool(self.get_parameter("interpolation").value)
            if use_bezier and use_interp:
                path_points = self.to_bezier(path_points, 0.5)
            memory_msg.data = False
        else:
            path_points = self.get_turn_memory_path()
            memory_msg.data = True

        self.turn_memory_pub.publish(memory_msg)

        self.publish_path(path_points)
        self.publish_line(1, self.pub_blue_cones, [0, 0, 1], blue_cones)
        self.publish_line(2, self.pub_yellow_cones, [1, 1, 0], yellow_cones)
        self.publish_line(3, self.pub_midpoints, [0, 1, 1], path_points)
        self.publish_pair_lines(4, self.pub_pair_lines, [1, 0, 1], pair_lines)



        return
    def order_cones_forward(self, cones, filter_radius):
        in_range = [
            c for c in cones
            if math.hypot(c[0], c[1]) > filter_radius
        ]
        return sorted(in_range, key=lambda c: c[0])

    def update_turn_memory_from_midpoints(self, midpoints):
        forward_points = [p for p in midpoints if p[0] > 1.0]
        if not forward_points:
            return

        sample = forward_points[:min(4, len(forward_points))]
        mean_y = sum(p[1] for p in sample) / len(sample)
        if abs(mean_y) > 0.1:
            self.turn_memory_sign = 1.0 if mean_y > 0.0 else -1.0

    def get_turn_memory_path(self):
        direction = 1.0 if self.turn_memory_sign >= 0.0 else -1.0
        return [
            [0.0, 0.0],
            [1.5, 0.25 * direction],
            [3.0, 0.7 * direction],
            [4.5, 1.35 * direction],
            [6.0, 2.2 * direction],
        ]

        


    def parse_cones_coords(self, cones):
        arr = []

        for cone in cones:
            arr.append([
                float(cone.point.x),
                float(cone.point.y)
            ])

        return arr

    def insert_initial_cone(self, con, cones):
        arr = cones[:]
        if con:
            arr.remove(con)

        if con:
            arr.insert(0, con)

        return arr
        
    def inRange(self, points, radius):
        return [point for point in points if (point[0] ** 2 + point[1] ** 2) ** 0.5 <= radius]


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

    def generate_imaginary_point(self, p1, p2, distance, direction):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        angle = math.atan2(dy, dx) + (math.pi / 2) * direction
        
        new_x = p2[0] + distance * math.cos(angle)
        new_y = p2[1] + distance * math.sin(angle)
        
        return float(new_x), float(new_y)

    def generate_imaginary_points(self, points, distance, direction):
        imaginary_points = []
        
        for i in range(len(points) - 1):
            x, y = self.generate_imaginary_point(points[i], points[i + 1], distance, direction)
            imaginary_points.append([x, y])
        
        return imaginary_points

    def calculate_midpoints(self, c1, c2):
        midpoints = []
        pair_lines = []

        if not c1 or not c2:
            return midpoints, pair_lines

        candidates = []
        for idx1, cone1 in enumerate(c1):
            for idx2, cone2 in enumerate(c2):
                dx = cone2.point.x - cone1.point.x
                dy = cone2.point.y - cone1.point.y
                d = math.hypot(dx, dy)
                if d <= self.threshold:
                    candidates.append((d, idx1, idx2))

        if not candidates:
            return midpoints, pair_lines

        candidates.sort(key=lambda item: item[0])

        used_c1 = set()
        used_c2 = set()
        for _, idx1, idx2 in candidates:
            if idx1 in used_c1 or idx2 in used_c2:
                continue

            used_c1.add(idx1)
            used_c2.add(idx2)

            cone1 = c1[idx1]
            cone2 = c2[idx2]

            mx = (cone1.point.x + cone2.point.x) / 2.0
            my = (cone1.point.y + cone2.point.y) / 2.0
            midpoints.append([float(mx), float(my)])
            pair_lines.append([
                [float(cone1.point.x), float(cone1.point.y)],
                [float(cone2.point.x), float(cone2.point.y)]
            ])

        # Zig-zag fallback pairing on unmatched cones to recover missing midpoints in turns.
        unmatched_c1 = [i for i in range(len(c1)) if i not in used_c1]
        unmatched_c2 = [i for i in range(len(c2)) if i not in used_c2]
        if unmatched_c1 and unmatched_c2:
            unmatched_c1.sort(key=lambda i: c1[i].point.x)
            zigzag_threshold = self.threshold * 1.8
            c2_reuse_count = {}
            max_reuse = 2

            for k, idx1 in enumerate(unmatched_c1):
                ranked = []
                for idx2 in unmatched_c2:
                    d = math.hypot(
                        c2[idx2].point.x - c1[idx1].point.x,
                        c2[idx2].point.y - c1[idx1].point.y,
                    )
                    if d <= zigzag_threshold:
                        ranked.append((d, idx2))

                if not ranked:
                    continue

                ranked.sort(key=lambda item: item[0])
                preferred_rank = 1 if (k % 2 == 1 and len(ranked) > 1) else 0

                choose_order = [preferred_rank, 0, 1, 2]
                candidate_idx2 = None
                for rank_idx in choose_order:
                    if rank_idx >= len(ranked):
                        continue
                    idx2 = ranked[rank_idx][1]
                    if c2_reuse_count.get(idx2, 0) >= max_reuse:
                        continue
                    candidate_idx2 = idx2
                    break

                if candidate_idx2 is None:
                    continue

                mx = (c1[idx1].point.x + c2[candidate_idx2].point.x) / 2.0
                my = (c1[idx1].point.y + c2[candidate_idx2].point.y) / 2.0
                midpoints.append([float(mx), float(my)])
                pair_lines.append([
                    [float(c1[idx1].point.x), float(c1[idx1].point.y)],
                    [float(c2[candidate_idx2].point.x), float(c2[candidate_idx2].point.y)]
                ])
                c2_reuse_count[candidate_idx2] = c2_reuse_count.get(candidate_idx2, 0) + 1

        return midpoints, pair_lines
 


    
     

    def state_callback(self, msg):
       
        con_sig = Int16()
        self.get_logger().info(" ami_state :" + str(msg.ami_state) + " self.state :" + str(msg.as_state))
        


        # if Acceleration and vehicle not stopped 
        if msg.ami_state == 11 and (msg.as_state != 4):
            
            #if the vehicle in ready state and not Finished
            if (msg.as_state == 2):

                    # logic for Accelearation check and stop 
                # if self.lap_count == 2 and  self.is_big_orange_cone == False:
                
                if (self.orange_cone_count >= 5) and ((self.blue_cone_count == 0) and (self.yellow_cone_count == 0)):
                    con_sig.data = 10
                    self.control_sig_pub.publish(con_sig)
        #If Skidpad and Vehicle not stopped 
        elif msg.ami_state == 12 and (msg.as_state != 4):
            #if the vehicle in ready state and not Finished
            if (msg.as_state == 2) and (msg.as_state != 4) :
                    # logic for Skidpad check and stop 
                if self.lap_count == 5 and  self.is_big_orange_cone == False:
                    con_sig.data = 20
                    self.control_sig_pub.publish(con_sig)
        #If  Autocross and vehicle not stopped     
        elif msg.ami_state == 13 and (msg.as_state != 4):
            #if the vehicle in ready state and not Finished
            if (msg.as_state == 2) and (msg.as_state != 4) :
                    # logic for Autocross check and stop 
                if self.lap_count == 2 and self.is_big_orange_cone == False:
                    con_sig.data = 30
                    self.control_sig_pub.publish(con_sig)
        #If  Track Drive  and vehicle not stopped     
        elif msg.ami_state == 14 and (msg.as_state != 4):
            #if the vehicle in ready state and not Finished
            if (msg.as_state == 2) and (msg.as_state != 4) :
                # logic for Track Drive check and stop 
                #rv23aao : change this before actual code 
                if self.lap_count == 4 and self.is_big_orange_cone == False:
                    con_sig.data = 40
                    self.control_sig_pub.publish(con_sig)
        #For small track  with Manual drive = 21          
        elif msg.ami_state == 21:
                # logic for Track Drive check and stop 
                if self.lap_count == 2 and self.is_big_orange_cone == False:
                    con_sig.data = 90
                    self.control_sig_pub.publish(con_sig)
 #       else:
 #               con_sig.data = 99
 #              self.control_sig_pub.publish(con_sig)



    def filter_points(self, ps, fov=90, render_distance=3):
        def angle_between_points(p1, p2):
            return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
        
        def distance_between_points(p1, p2):
            return ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
        
        sorted_mps = []
        remaining = ps[:]

        if len(remaining) == 0: return sorted_mps
        
        current_point = remaining.pop(0)
        sorted_mps.append(current_point)
        current_orientation = None

        while remaining:
            if current_orientation is not None:
                candidates = []
                for point in remaining:
                    angle_to_point = angle_between_points(current_point, point)
                    distance_to_point = distance_between_points(current_point, point)
                    
                    if (
                        abs((angle_to_point - current_orientation + 180) % 360 - 180) <= fov / 2
                        and distance_to_point <= render_distance
                    ):
                        candidates.append(point)

                if not candidates:
                    break

                next_point = min(candidates, key=lambda p: distance_between_points(current_point, p))
            else:
                candidates = [p for p in remaining if distance_between_points(current_point, p) <= render_distance]
                if not candidates:
                    break 
                next_point = min(candidates, key=lambda p: distance_between_points(current_point, p))
            
            current_orientation = angle_between_points(current_point, next_point)

            current_point = next_point
            sorted_mps.append(next_point)
            remaining.remove(next_point)

        return sorted_mps
    
    def calc_angle_of_turn(self, p1, p2, p3):
        def angle_between_vectors(v1, v2):
            dot_product = np.dot(v1, v2)
            magnitude_v1 = np.linalg.norm(v1)
            magnitude_v2 = np.linalg.norm(v2)
            angle = np.arccos(dot_product / (magnitude_v1 * magnitude_v2))
            cross_product = np.cross(v1, v2)
            return np.degrees(angle) if cross_product >= 0 else -np.degrees(angle)

        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        return angle_between_vectors(v1, v2)

    def to_bezier(self, mp, distance):
        def bezier(t, points):
            n = len(points) - 1
            return sum(
                (np.math.comb(n, i) * (1 - t) ** (n - i) * t ** i * np.array(points[i]))
                for i in range(n + 1)
            )

        def arc_length(points, steps=1000):
            t_values = np.linspace(0, 1, steps)
            curve_points = [bezier(t, points) for t in t_values]
            distances = [np.linalg.norm(curve_points[i] - curve_points[i - 1]) for i in range(1, len(curve_points))]
            cumulative_distances = np.cumsum(distances)
            return cumulative_distances, curve_points

        cumulative_distances, curve_points = arc_length(mp)

        total_length = cumulative_distances[-1]

        target_distances = np.arange(0, total_length, distance)
        bezier_curve = []
        idx = 0

        for target in target_distances:
            while idx < len(cumulative_distances) and cumulative_distances[idx] < target:
                idx += 1
            if idx >= len(cumulative_distances):
                break

            if idx == 0:
                bezier_curve.append(curve_points[0])
            else:
                prev_point = curve_points[idx - 1]
                next_point = curve_points[idx]
                prev_dist = cumulative_distances[idx - 1]
                next_dist = cumulative_distances[idx]
                alpha = (target - prev_dist) / (next_dist - prev_dist)
                interpolated_point = prev_point + alpha * (next_point - prev_point)
                bezier_curve.append(interpolated_point)

        return [point.tolist() for point in bezier_curve]


    def toFixedBezier(self, points, num_points):
        if len(points) < 2:
            return points

        bezier_points = []
        
        for t in range(num_points):
            t = t / (num_points - 1)
            
            temp_points = points[:]
            
            while len(temp_points) > 1:
                temp_points = [
                    [(1 - t) * p1[0] + t * p2[0], (1 - t) * p1[1] + t * p2[1]]
                    for p1, p2 in zip(temp_points[:-1], temp_points[1:])
                ]
            
            bezier_points.append(temp_points[0])

        return bezier_points

    def publish_path(self, midpoints):
        waypoint_array = WaypointArrayStamped()
        waypoint_array.header.frame_id = "base_footprint"
        waypoint_array.header.stamp = self.get_clock().now().to_msg()

        for p in midpoints:
            point = Point(x=p[0], y=p[1])
            waypoint = Waypoint(position=point)
            waypoint_array.waypoints.append(waypoint)

        self.track_line_pub.publish(waypoint_array)

    def publish_line(self, id, publisher, rgb, data, scale=0.1):
        if(not len(data)): return
        if(type(data[0]) != list):
            data = [data]

        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.action = Marker.ADD
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.color.a = 1.0
        marker.color.r = float(rgb[0])
        marker.color.g = float(rgb[1])
        marker.color.b = float(rgb[2])
        marker.id = id
        marker.scale.x = scale
        marker.scale.y = scale
        marker.ns = "line_stip"
        for line in data:
            marker.points.append(Point(x=line[0], y=line[1]))

        publisher.publish(marker)

    def publish_pair_lines(self, id, publisher, rgb, pairs, scale=0.05):
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.action = Marker.ADD
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_LIST
        marker.color.a = 1.0
        marker.color.r = float(rgb[0])
        marker.color.g = float(rgb[1])
        marker.color.b = float(rgb[2])
        marker.id = id
        marker.scale.x = scale
        marker.scale.y = scale
        marker.ns = "pair_lines"

        for pair in pairs:
            marker.points.append(Point(x=pair[0][0], y=pair[0][1]))
            marker.points.append(Point(x=pair[1][0], y=pair[1][1]))

        publisher.publish(marker)





def main():
    rclpy.init(args=None)
    node = Planner("local_planner")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
