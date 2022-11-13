from cmath import sin
from geometry_msgs.msg import Point
# from typing import _KT_co
from eufs_msgs.msg import WaypointArrayStamped, CarState
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from rclpy.node import Node
import math

import numpy as np

class Control(Node):
    def __init__(self, name):
        super().__init__(name)
        self.speed = 0
        self.error_buffer = [0]
        self.period = 0.04      # the time between updates to the path

        # Declare ROS parameters
        self.look_ahead = self.declare_parameter("look_ahead", 4.0).value
        self.L = self.declare_parameter("L", 1.5).value
        self.K_p = self.declare_parameter("K_p", 1.0).value
        self.K_i = self.declare_parameter("K_i", 1.0).value
        self.K_d = self.declare_parameter("K_d", 1.0).value
        self.max_lat_acc = self.declare_parameter("max_lat_acc", 5.0).value
        self.safe_speed = self.declare_parameter("safe_speed", 1.5).value
        self.max_speed = self.declare_parameter("max_speed", 4.5).value
        self.buffer_len = self.declare_parameter("buffer_len", 30).value

        # Create subscribers
        self.path_sub = self.create_subscription(WaypointArrayStamped, "/trajectory", self.path_callback, 1)
        self.car_state_sub = self.create_subscription(CarState, "/ground_truth/state", self.state_callback, 1)

        # Create publishers
        self.comand_pub = self.create_publisher(AckermannDriveStamped, "/cmd", 1)
        self.viz_pub = self.create_publisher(Marker, "/control/viz", 1)
        self.visualization_look_ahead_index = self.create_publisher(Marker, "/control/Index", 1)

    def state_callback(self, msg):
        self.speed = msg.twist.twist.linear.x

    def path_callback(self, msg):
        path = self.convert(msg, "np")  # if you remove the "np" parameter the path will be a 2d array [[x1,y1], ...]
        if len(path) == 0: return

        # Index of the waypoint to the look ahead distance
        look_ahead_index = self.get_look_ahead_index(path)

        # Steering control
        steering_cmd = self.get_steering(path, look_ahead_index)

        # Speed control
        speed_target = self.get_speed_target(path, look_ahead_index)

        # PID control for acceleration
        acceleration_cmd = self.get_acceleration(speed_target)

        # Publish commands
        self.pubish_command(acceleration_cmd, steering_cmd)
        self.publish_visualisation(acceleration_cmd, steering_cmd)

    def get_look_ahead_index(self, path):
        """
        IMPLEMENT YOURSELF

        :param path: array of complex numbers
        :return: index of waypoint closest to look ahead distance
        """

        # self.get_logger().info("---------->  path :")
        # self.get_logger().info(str(path))

        for index in path:
            if index.real > self.look_ahead :
                look_ahead_index = index
                break;        

        # self.get_logger().info("---------->  look_ahead_index :")
        # self.get_logger().info(str(look_ahead_index))
        self.publish_look_ahead_index(look_ahead_index)

        return look_ahead_index

    def get_steering(self, path, look_ahead_ind):
        """
        IMPLEMENT YOURSELF
        note: the wheelbase of the car L is saved in the self.L variable
        :param path: array of complex numbers
        :param look_ahead_ind:
        :return: steering angle to be sent to the car
        """

        # steering_angle1 = math.atan((2*self.L * sin(look_ahead_ind.imag))/look_ahead_ind.real)
        steering_angle = np.arctan((2*self.L * sin(look_ahead_ind.imag))/look_ahead_ind.real)

        # self.get_logger().info("---------->  steering_angle1 :")
        # self.get_logger().info(str(steering_angle1))
        self.get_logger().info("---------->  steering_angle2 :")
        self.get_logger().info(str(steering_angle))

        return steering_angle.real

    def get_speed_target(self, path, look_ahead_ind):
        """
        IMPLEMENT YOURSELF
        note: You might want to use the max_lat_acc variable to limit lateral acceleration
        and max_speed to limit the maximum speed
        :param path: array of complex numbers
        :param look_ahead_ind:
        :return: speed we want to reach
        """

        # self.max_lat_acc
        # target_speed = 
        k = 1
        path_radius = 1/ k
        target_speed = math.sqrt (abs(path_radius) * self.max_lat_acc)

        self.get_logger().info("---------->  target_speed :")
        self.get_logger().info(str(target_speed))

        return target_speed

    def get_acceleration(self, speed_target):
        """
        IMPLEMENT YOURSELF
        Note: the current speed of the car is saved in self.speed
        the PID gains are saved in self.K_p, self.K_i, self.K_d
        :param speed_target: speed we want to achieve
        :return: acceleration command to be sent to the car
        """

        # desired state r(t) =  velocity we want to achieve ( speed_target )
        # current state y(t) =  velocity from localization (self.speed : for now)
        # correction u(t) = acceleration we apply to the car (new_acc)

        # P = 
        # I = 
        # D = 

        # new_acc = P + I + D

        new_acc = self.safe_speed

        return new_acc

    def pubish_command(self, acceleration, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pure_pursuit"
        msg.drive.steering_angle = steering
        msg.drive.acceleration = acceleration

        self.comand_pub.publish(msg)



    def publish_visualisation(self, acceleration, steering_angle):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_footprint"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = 3.0
        marker.pose.position.y = 4.0
        marker.pose.position.z = 1.0
        marker.id = 0
        marker.ns = "controls"
        marker.id = 0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.5
        marker.text = f" Speed: {round(self.speed, 1)} \n " \
                      f"Acceleration: {round(acceleration, 1)} \n " \
                      f"Steering: {round(steering_angle, 1)}"

        self.viz_pub.publish(marker)

    def publish_look_ahead_index(self, look_ahead_index):
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.action = Marker.ADD
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.id = 1
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.ns = "look_ahead_index"
        marker.points.append(Point(x=look_ahead_index.real, y=look_ahead_index.imag))

        self.visualization_look_ahead_index.publish(marker)


    def convert(self, waypoints, struct = ''):
        """
        Converts a cone array message into a np array of complex or 2d list
        :param cones: ConeArrayWithCovariance
        :param struct: Type of output list
        :return:
        """
        if struct == "np":
            return [p.position.x + 1j * p.position.y for p in waypoints.waypoints]
        else:
            return [[p.position.x, p.position.y] for p in waypoints.waypoints]


def main():
    rclpy.init(args=None)
    node = Control("pure_pursuit")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
