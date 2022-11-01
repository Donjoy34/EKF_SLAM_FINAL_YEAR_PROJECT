from turtle import distance
from eufs_msgs.msg import WaypointArrayStamped, Waypoint, ConeArrayWithCovariance, CarState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rclpy.node import Node
import rclpy
import math

import numpy as np
from scipy import interpolate

class Planner(Node):
    def __init__(self, name):
        super().__init__(name)
        # Declare ROS parameters
        self.threshold = self.declare_parameter("threshold", 6.0).value

        # Create subscribers
        self.cones_sub = self.create_subscription(ConeArrayWithCovariance, "/fusion/cones", self.cones_callback, 1)

        # Create publishers
        self.track_line_pub = self.create_publisher(WaypointArrayStamped, "/trajectory", 1)
        self.visualization_pub = self.create_publisher(Marker, "/planner/viz", 1)

    def cones_callback(self, msg):
        blue_cones = self.convert(msg.blue_cones)
        yellow_cones = self.convert(msg.yellow_cones)
        orange_cones = self.to_2d_list(self.convert(msg.orange_cones))
        uncolored_cones = self.convert(msg.unknown_color_cones)

        # add uncolored lidar cones to the appropriate sides
        close_uncolored = uncolored_cones[np.abs(uncolored_cones) < 4]
        blue_cones = self.to_2d_list(np.concatenate((blue_cones, close_uncolored[close_uncolored.imag > 0])))
        yellow_cones = self.to_2d_list(np.concatenate((yellow_cones, close_uncolored[close_uncolored.imag < 0])))

        midpoints = self.find_midpoints(blue_cones, yellow_cones, orange_cones)
        midpoints = self.sort_midpoints(midpoints)

        # Convert back to complex
        midpoints_c = np.array([])
        for m in midpoints:

            # self.get_logger().info("List index:")
            # self.get_logger().info(str(m))
            midpoints_c = np.append(midpoints_c, m[0] + 1j * m[1])

        if len(midpoints_c) == 0:
            return

        # Here we interpolate the path to artificially increase the number of midpoints so that the controllers
        # have more information to work with you don't need to worry about this step
        try:
            tck, _, = interpolate.splprep(np.array([midpoints_c.real, midpoints_c.imag]), k=3, s=100)
            x_i, y_i = interpolate.splev(np.linspace(0, 1, 10), tck)
            midpoints_c = np.array(x_i + 1j * y_i)
        except:
            self.get_logger().info("Failed to interpolate")

        self.publish_path(midpoints_c)
        self.publish_visualisation(midpoints_c)

    def find_midpoints(self, blue_cones, yellow_cones, orange_cones):
        """
        IMPLEMENT YOURSELF
        Find the midpoints along the track
        :param blue_cones: cone positions
        :param yellow_cones:
        :param orange_cones:
        :return: list of midpoints
        """
        if len(blue_cones) <= len(yellow_cones):
            num_cones = len(blue_cones)
        else:
            num_cones = len(yellow_cones)

        midpoints = []
        for cone_pos in range(0,num_cones):
            midpoints.append([((blue_cones[cone_pos][0] + yellow_cones[cone_pos][0]) / 2 ) , ((blue_cones[cone_pos][1] + yellow_cones[cone_pos][1]) / 2 )])

        return midpoints

    def sort_midpoints(self, midpoints):
        """
        IMPLEMENT YOURSELF
        Sort the midpoints to so that each consecutive midpoints is further from the car along the path
        :param midpoints:
        :return: sorted midpoints
        """
        cone_dict = {}
        for cones in range(0,len(midpoints)):
            cone_dict.update({math.sqrt(((midpoints[cones][0]- 0) ** 2) + (((midpoints[cones][1]- 0) ** 2))) : midpoints[cones]})

        sorted_dict = {k : cone_dict[k] for k in sorted(cone_dict)}

        sorted_midpoints = list(sorted_dict.values())

        return sorted_midpoints

    def publish_path(self, midpoints):
        waypoint_array = WaypointArrayStamped()
        waypoint_array.header.frame_id = "base_footprint"
        waypoint_array.header.stamp = self.get_clock().now().to_msg()

        for p in midpoints:
            point = Point(x=p.real, y=p.imag)
            waypoint = Waypoint(position=point)
            waypoint_array.waypoints.append(waypoint)

        self.track_line_pub.publish(waypoint_array)

    def publish_visualisation(self, midpoints):
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.action = Marker.ADD
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.id = 0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.ns = "midpoints"
        for midpoint in midpoints:
            marker.points.append(Point(x=midpoint.real, y=midpoint.imag))

        self.visualization_pub.publish(marker)

    def convert(self, cones):
        """
        Converts a cone array message into a np array of complex
        """
        return np.array([c.point.x + 1j * c.point.y for c in cones])

    def to_2d_list(self, arr):
        """
        Convert an array of complex numbers to a 2d array [[x1,y1], ...]
        :param arr: array of complex
        :return: 2d list
        """
        return [[x.real, x.imag] for x in arr]

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
