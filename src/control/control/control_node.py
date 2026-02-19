from geometry_msgs.msg import Point
# from typing import _KT_co
from eufs_msgs.msg import WaypointArrayStamped, CarState
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from rclpy.node import Node
import math

class Control(Node):
    def __init__(self, name):
        super().__init__(name)
        self.current_speed = 0
        self.error_buffer = [0]
        self.period = 0.04      # the time between updates to the path

        # Declare ROS parameters
        self.look_ahead = self.declare_parameter("look_ahead", 3.0).value
        self.L = self.declare_parameter("L", 1.5).value
        self.K_p = self.declare_parameter("K_p", 1.0).value
        self.K_i = self.declare_parameter("K_i", 1.0).value
        self.K_d = self.declare_parameter("K_d", 1.0).value
        self.steering_gain = self.declare_parameter("steering_gain", 1.1).value
        self.max_steering = self.declare_parameter("max_steering", 0.5).value
        self.min_turn_speed = self.declare_parameter("min_turn_speed", 0.8).value
        self.turn_slow_angle = self.declare_parameter("turn_slow_angle", 0.35).value
        self.max_turn_angle = self.declare_parameter("max_turn_angle", 0.9).value
        self.turn_memory_hold_sec = self.declare_parameter("turn_memory_hold_sec", 1.0).value
        self.turn_memory_min_angle = self.declare_parameter("turn_memory_min_angle", 0.25).value
        self.turn_memory_blend = self.declare_parameter("turn_memory_blend", 0.35).value
        self.turn_memory_speed = self.declare_parameter("turn_memory_speed", 0.9).value
        self.turn_memory_waypoint_min = self.declare_parameter("turn_memory_waypoint_min", 6).value
        self.turn_memory_waypoint_max = self.declare_parameter("turn_memory_waypoint_max", 7).value
        self.blind_recovery_hold_sec = self.declare_parameter("blind_recovery_hold_sec", 1.2).value
        self.blind_recovery_speed = self.declare_parameter("blind_recovery_speed", 0.8).value
        self.blind_recovery_min_steer = self.declare_parameter("blind_recovery_min_steer", 0.18).value
        self.max_lat_acc = self.declare_parameter("max_lat_acc", 5.0).value
        self.safe_speed = self.declare_parameter("safe_speed", 1.5).value
        self.max_speed = self.declare_parameter("max_speed", 4.5).value
        self.buffer_len = self.declare_parameter("buffer_len", 30).value

        # Create subscribers
        self.path_sub = self.create_subscription(WaypointArrayStamped, "/trajectory", self.path_callback, 1)
        self.car_state_sub = self.create_subscription(CarState, "/ground_truth/state", self.state_callback, 1)

        # Create publishers
        self.command_pub = self.create_publisher(AckermannDriveStamped, "/cmd", 1)
        self.viz_pub = self.create_publisher(Marker, "/control/viz", 1)
        self.visualization_look_ahead_index = self.create_publisher(Marker, "/control/Index", 1)

        self.acc_previous_error = 0
        self.acc_integral = 0
        self.turn_memory_sign = 0.0
        self.turn_memory_strength = 0.0
        self.turn_memory_until_ns = 0
        self.memory_mode_active = False
        self.last_valid_steering = 0.0
        self.last_path_seen_ns = 0

    def state_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def path_callback(self, msg):
        path = self.convert(msg, "np")  # if you remove the "np" parameter the path will be a 2d array [[x1,y1], ...]
        now_ns = self.get_clock().now().nanoseconds

        if len(path) == 0:
            if self.should_use_blind_recovery(now_ns):
                recovery_steering = self.last_valid_steering
                if abs(recovery_steering) < self.blind_recovery_min_steer and self.turn_memory_sign != 0.0:
                    recovery_steering = self.turn_memory_sign * self.blind_recovery_min_steer

                recovery_steering = max(-self.max_steering, min(self.max_steering, recovery_steering))
                speed_target = self.blind_recovery_speed
                acceleration_cmd = self.get_acceleration(speed_target)
                self.memory_mode_active = True
                self.publish_command(acceleration_cmd, recovery_steering, speed_target)
                self.publish_visualisation(acceleration_cmd, recovery_steering)
            else:
                self.memory_mode_active = False
            return

        self.last_path_seen_ns = now_ns

        # Index of the waypoint to the look ahead distance
        look_ahead_index = self.get_look_ahead_index(path)

        # Steering control
        steering_cmd = self.get_steering(path, look_ahead_index)
        self.update_turn_memory(look_ahead_index)
        waypoint_count = len(path)
        memory_eligible = self.turn_memory_waypoint_min <= waypoint_count <= self.turn_memory_waypoint_max
        memory_steering = self.get_turn_memory_steering(memory_eligible)
        if abs(memory_steering) > 1e-3:
            steering_cmd = steering_cmd * (1.0 - self.turn_memory_blend) + memory_steering * self.turn_memory_blend
            steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))

        self.last_valid_steering = steering_cmd

        # Speed control
        speed_target = self.get_speed_target(path, look_ahead_index)

        # PID control for acceleration
        acceleration_cmd = self.get_acceleration(speed_target)


        self.get_logger().info(f"speed: {self.current_speed}, steering: {steering_cmd}, speed: {speed_target}, acc: {acceleration_cmd}", throttle_duration_sec=.3)

        # Publish commands
        self.publish_command(acceleration_cmd, steering_cmd, speed_target)
        self.publish_visualisation(acceleration_cmd, steering_cmd)

    def get_look_ahead_index(self, path):
        """
        :param path: array of complex numbers
        :return: index of waypoint closest to look ahead distance
        """

        # self.get_logger().info("---------->  path :", throttle_duration_sec=.3)
        # self.get_logger().info(str(path), throttle_duration_sec=.3)
        desired_lookahead_index = 2
        look_ahead_index = 1 + 0j

        # for index in path:
        #     if index.real > self.look_ahead :
        #         look_ahead_index = index
        #         break;   
        if len(path) == 0:
            return look_ahead_index
        forward_points = [p for p in path if p.real > 0.0]
        if not forward_points:
            self.publish_look_ahead_index(path[-1])
            return path[-1]

        first_forward = forward_points[0]
        first_forward_angle = abs(math.atan2(first_forward.imag, max(first_forward.real, 0.1)))

        if first_forward_angle > self.turn_slow_angle:
            desired_lookahead_index = 0

        if len(forward_points) > desired_lookahead_index:
            selected = forward_points[desired_lookahead_index]
            self.publish_look_ahead_index(selected)
            return selected

        self.publish_look_ahead_index(forward_points[-1])
        return forward_points[-1]


    def get_steering(self, path, look_ahead_ind):
        """
        note: the wheelbase of the car L is saved in the self.L variable
        :param path: array of complex numbers
        :param look_ahead_ind:
        :return: steering angle to be sent to the car
        """
        desired_angle = math.atan2(look_ahead_ind.imag, max(look_ahead_ind.real, 0.1))
        desired_angle *= self.steering_gain
        desired_angle = max(-self.max_steering, min(self.max_steering, desired_angle))
        self.get_logger().info(f"steering: {desired_angle}")

        return desired_angle

    
        # kp = 4.5
        # ki = 1.5
        # kd = 1.5
        # error = np.angle(look_ahead_ind)   # +ve angle towards left and -ve steering angle towards right
        # self.acc_integral = error
        # derivative = error - self.acc_previous_error
        # self.acc_previous_error = error

        # desired_angle = kp * error + ki * self.acc_integral + kd * derivative
        # self.get_logger().info(f"steering: {desired_angle}")

        # return desired_angle

    def get_speed_target(self, path, look_ahead_ind):
        """
        note: You might want to use the max_lat_acc variable to limit lateral acceleration
        and max_speed to limit the maximum speed
        :param path: array of complex numbers
        :param look_ahead_ind:
        :return: speed we want to reach
        """
        turn_angle = abs(math.atan2(look_ahead_ind.imag, max(look_ahead_ind.real, 0.1)))

        if turn_angle <= self.turn_slow_angle:
            target_speed = self.safe_speed
        else:
            ratio = min(1.0, (turn_angle - self.turn_slow_angle) / max(1e-3, self.max_turn_angle - self.turn_slow_angle))
            target_speed = self.safe_speed - ratio * (self.safe_speed - self.min_turn_speed)

        target_speed = max(self.min_turn_speed, min(target_speed, self.max_speed))

        if self.memory_mode_active:
            target_speed = min(target_speed, self.turn_memory_speed)

        return target_speed

    def update_turn_memory(self, look_ahead_ind):
        turn_angle = math.atan2(look_ahead_ind.imag, max(look_ahead_ind.real, 0.1))
        if abs(turn_angle) < self.turn_memory_min_angle:
            return

        self.turn_memory_sign = 1.0 if turn_angle > 0.0 else -1.0
        self.turn_memory_strength = min(self.max_steering, abs(turn_angle) * self.steering_gain)
        now_ns = self.get_clock().now().nanoseconds
        self.turn_memory_until_ns = now_ns + int(self.turn_memory_hold_sec * 1e9)

    def get_turn_memory_steering(self, memory_eligible):
        if not memory_eligible:
            self.memory_mode_active = False
            return 0.0

        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= self.turn_memory_until_ns and self.turn_memory_sign != 0.0:
            self.memory_mode_active = True
            return self.turn_memory_sign * self.turn_memory_strength

        self.memory_mode_active = False
        return 0.0

    def should_use_blind_recovery(self, now_ns):
        if self.last_path_seen_ns == 0:
            return False
        return (now_ns - self.last_path_seen_ns) <= int(self.blind_recovery_hold_sec * 1e9)

    def get_acceleration(self, speed_target):
        """
        Note: the current speed of the car is saved in self.speed
        the PID gains are saved in self.K_p, self.K_i, self.K_d
        :param speed_target: speed we want to achieve
        :return: acceleration command to be sent to the car
        """
        if speed_target <= 0.0:
            return 0.0

        Kp = 0.8
        acc_error = speed_target - self.current_speed
        new_acc = Kp * acc_error

        return max(-1.0, min(2.0, new_acc))

    def publish_command(self, acceleration, steering, speed_target):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pure_pursuit"
        msg.drive.steering_angle = steering
        msg.drive.acceleration = acceleration
        msg.drive.speed = speed_target

        self.command_pub.publish(msg)

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
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.5
        marker.text = f" Speed: {round(self.current_speed, 1)} \n " \
                      f"Acceleration: {round(acceleration, 1)} \n " \
                      f"Steering: {round(steering_angle, 1)} \n " \
                      f"Memory: {'ON' if self.memory_mode_active else 'OFF'}"

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
