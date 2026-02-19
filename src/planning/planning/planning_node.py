import math
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from eufs_msgs.msg import ConeArrayWithCovariance, Waypoint, WaypointArrayStamped


class Planner(Node):
    def __init__(self, name, executor):
        super().__init__(name)
        self.executor = executor
        self.half_track_width = 1.5
        self.pairing_distance = 5.0

        self.create_subscription(ConeArrayWithCovariance, '/cones', self.on_cones, 1)
        self.midpoint_pub = self.create_publisher(WaypointArrayStamped, '/midpoints', 10)
        self.trajectory_pub = self.create_publisher(WaypointArrayStamped, '/trajectory', 10)
        self.get_logger().info('planning_node active: using built-in midpoint planner')

    def on_cones(self, msg):
        blue_cones = list(msg.blue_cones)
        yellow_cones = list(msg.yellow_cones)

        for cone in msg.big_orange_cones:
            if cone.point.y < 0.0:
                yellow_cones.append(cone)
            else:
                blue_cones.append(cone)

        midpoints = self.calculate_midpoints_with_fallback(blue_cones, yellow_cones)
        if not midpoints:
            return

        ordered = self.order_forward(midpoints)
        if ordered and ordered[0][0] > 0.2:
            ordered.insert(0, [0.0, 0.0])

        self.publish_waypoints(self.midpoint_pub, ordered, speed=0.0)
        self.publish_waypoints(self.trajectory_pub, ordered, speed=2.5)

    def publish_waypoints(self, publisher, points, speed):
        msg = WaypointArrayStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'

        for point_xy in points:
            waypoint = Waypoint()
            waypoint.position.x = float(point_xy[0])
            waypoint.position.y = float(point_xy[1])
            waypoint.position.z = 0.0
            waypoint.speed = speed
            waypoint.suggested_steering = 0.0
            msg.waypoints.append(waypoint)

        publisher.publish(msg)

    def calculate_midpoints_with_fallback(self, blue_cones, yellow_cones):
        midpoints = []

        if blue_cones and yellow_cones:
            pairs, _, _ = self.match_cones(blue_cones, yellow_cones, float('inf'))
            for blue_cone, yellow_cone in pairs:
                midpoints.append([
                    (blue_cone.point.x + yellow_cone.point.x) / 2.0,
                    (blue_cone.point.y + yellow_cone.point.y) / 2.0,
                ])
            return midpoints

        if blue_cones and not yellow_cones:
            side_sign = self.estimate_side_sign(blue_cones)
            for blue_cone in blue_cones:
                midpoints.append(self.fallback_from_single_cone(blue_cone, blue_cones, [], side_sign))
            return midpoints

        if yellow_cones and not blue_cones:
            side_sign = self.estimate_side_sign(yellow_cones)
            for yellow_cone in yellow_cones:
                midpoints.append(self.fallback_from_single_cone(yellow_cone, yellow_cones, [], side_sign))
            return midpoints

        return midpoints

    def match_cones(self, side_a, side_b, max_distance):
        used_b = set()
        pairs = []
        unmatched_a = []

        for cone_a in side_a:
            best_index = None
            best_distance = max_distance

            for index_b, cone_b in enumerate(side_b):
                if index_b in used_b:
                    continue
                distance = self.distance(cone_a.point.x, cone_a.point.y, cone_b.point.x, cone_b.point.y)
                if distance < best_distance:
                    best_distance = distance
                    best_index = index_b

            if best_index is None:
                unmatched_a.append(cone_a)
            else:
                used_b.add(best_index)
                pairs.append((cone_a, side_b[best_index]))

        unmatched_b = [cone_b for index_b, cone_b in enumerate(side_b) if index_b not in used_b]
        return pairs, unmatched_a, unmatched_b

    def fallback_from_single_cone(self, cone, same_side_cones, opposite_side_cones, side_sign=None):
        tangent_x, tangent_y = self.estimate_tangent(cone, same_side_cones)
        normal_1 = (-tangent_y, tangent_x)
        normal_2 = (tangent_y, -tangent_x)

        chosen_normal = self.pick_inward_normal(cone, normal_1, normal_2, opposite_side_cones, side_sign)
        normal_norm = math.hypot(chosen_normal[0], chosen_normal[1])
        if normal_norm < 1e-6:
            chosen_normal = (0.0, -1.0 if cone.point.y >= 0.0 else 1.0)
            normal_norm = 1.0

        nx = chosen_normal[0] / normal_norm
        ny = chosen_normal[1] / normal_norm
        return [
            cone.point.x + nx * self.half_track_width,
            cone.point.y + ny * self.half_track_width,
        ]

    def pick_inward_normal(self, cone, normal_1, normal_2, opposite_side_cones, side_sign=None):
        if opposite_side_cones:
            mean_x = sum(c.point.x for c in opposite_side_cones) / len(opposite_side_cones)
            mean_y = sum(c.point.y for c in opposite_side_cones) / len(opposite_side_cones)
            toward_other = (mean_x - cone.point.x, mean_y - cone.point.y)
            dot_1 = normal_1[0] * toward_other[0] + normal_1[1] * toward_other[1]
            dot_2 = normal_2[0] * toward_other[0] + normal_2[1] * toward_other[1]
            return normal_1 if dot_1 >= dot_2 else normal_2

        if side_sign is None:
            side_sign = 1.0 if cone.point.y >= 0.0 else -1.0

        desired_y_direction = -side_sign
        score_1 = normal_1[1] * desired_y_direction
        score_2 = normal_2[1] * desired_y_direction
        if score_1 == score_2:
            y_after_1 = abs(cone.point.y + normal_1[1] * self.half_track_width)
            y_after_2 = abs(cone.point.y + normal_2[1] * self.half_track_width)
            return normal_1 if y_after_1 <= y_after_2 else normal_2

        return normal_1 if score_1 > score_2 else normal_2

    def estimate_side_sign(self, cones):
        if not cones:
            return 1.0
        mean_y = sum(c.point.y for c in cones) / len(cones)
        return 1.0 if mean_y >= 0.0 else -1.0

    def estimate_tangent(self, cone, same_side_cones):
        nearest = None
        nearest_distance = float('inf')

        for other in same_side_cones:
            if other is cone:
                continue
            dx = other.point.x - cone.point.x
            dy = other.point.y - cone.point.y
            d = math.hypot(dx, dy)
            if d < 1e-6:
                continue
            if d < nearest_distance:
                nearest_distance = d
                nearest = other

        if nearest is None:
            return 1.0, 0.0

        tx = nearest.point.x - cone.point.x
        ty = nearest.point.y - cone.point.y
        norm = math.hypot(tx, ty)
        if norm < 1e-6:
            return 1.0, 0.0
        return tx / norm, ty / norm

    def order_forward(self, points):
        filtered = [p for p in points if math.hypot(p[0], p[1]) > 0.3]
        return sorted(filtered, key=lambda p: (p[0], abs(p[1])))

    def distance(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    planner = Planner('planning_node', executor)
    executor.add_node(planner)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
