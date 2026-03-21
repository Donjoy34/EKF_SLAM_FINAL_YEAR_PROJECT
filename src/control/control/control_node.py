from eufs_msgs.msg import WaypointArrayStamped, CanState, ConeArrayWithCovariance, WheelSpeeds, CarState
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import rclpy
from rclpy.node import Node
import math
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Bool



class PIDController:
    def __init__(self, kp, ki, kd, output_min=0.0, output_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._prev_error = 0.0
        self._integral = 0.0
        self.output_min = output_min
        self.output_max = output_max


    def reset(self):
        self._prev_error = 0.0
        self._integral = 0.0

    def update(self, error, dt):
        # Proportional
        p = self.kp * error
        # Integral
        self._integral += error * dt
        i = self.ki * self._integral
        # Derivative
        d = self.kd * (error - self._prev_error) / dt if dt > 0.0 else 0.0
        self._prev_error = error

        raw = p + i + d
        return max(self.output_min, min(self.output_max, raw))

class Control(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # Time-keeping for dt
        self.prev_time = self.get_clock().now().nanoseconds * 1e-9

        self.stop_triggered = False
        self.steer_rad = 0.0
        self.steer_change_cmd = 0.0
        self.ws_ami_state = 0
        self.ws_as_state = 0
        self.ws_can_steering = 0.0

        # Parameters
        self.declare_parameter("static_lookahead_idx", 6)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("Kp_acc", 0.00)
        self.declare_parameter("Ki_acc", 0.00)
        self.declare_parameter("Kd_acc", 0.00)
        self.declare_parameter("wheel_radius_m", 0.2525)
        self.declare_parameter("speed_timeout_sec", 0.5)
        # ±60° steering limit, back off at ±59°
        self.declare_parameter("steer_limit_deg", 60.0)
        self.declare_parameter("steer_cap_deg",   59.0)

        self.min_speed  = self.get_parameter("min_speed").value
        self.max_speed  = self.get_parameter("max_speed").value
        self.wheel_radius_m = self.get_parameter("wheel_radius_m").value
        self.speed_timeout_sec = self.get_parameter("speed_timeout_sec").value
        self._steer_lim = self.get_parameter("steer_limit_deg").value
        self._steer_cap = self.get_parameter("steer_cap_deg").value

        self.mission_completed_pub = self.create_publisher(Bool,"/ros_can/mission_completed",1)
        self.driving_flag_pub = self.create_publisher(Bool,"/state_machine/driving_flag",1)


        # PID for steering‐based accel command
        Kp = self.get_parameter("Kp_acc").value
        Ki = self.get_parameter("Ki_acc").value
        Kd = self.get_parameter("Kd_acc").value
        self.steer_pid = PIDController(Kp, Ki, Kd,
                                       output_min=0.0, output_max=1.0)

        # ROS interfaces
        self.create_subscription(WaypointArrayStamped, "/trajectory", self.path_callback, 1)
        self.create_subscription(CarState, "/ground_truth/state", self.state_callback, 1)
        #Create subscribers
        self.create_subscription(ConeArrayWithCovariance, "/cones", self.cones_callback, 1)

        #------------------------------------------------------------------

        self.car_state_sub = self.create_subscription(Int16, "/planner/ConSig", self.consig_callback, 1)
        self.car_state_sub1 = self.create_subscription(CanState, "/ros_can/state", self.can_state_callback, 1)
        self.car_state_sub2 = self.create_subscription(WheelSpeeds, "/ros_can/wheel_speeds", self.wheel_speed_callback, 1)





        self.cmd_pub       = self.create_publisher(AckermannDriveStamped, "/cmd",        1)
        self.viz_pub       = self.create_publisher(Marker,                  "/control/viz",   1)
        self.index_viz_pub = self.create_publisher(Marker,                  "/control/Index", 1)
        self.create_subscription(Bool, "/planner/turn_memory_active", self.turn_memory_callback, 1)

        self.speed = 0.0
        self.last_speed_update_ns = 0
        self.speed_source = "NONE"
        self.turn_memory_active = False
        self.last_pid_out = 0.0


    def state_callback(self, msg: CarState):
        self.speed = abs(float(msg.twist.twist.linear.x))
        self.last_speed_update_ns = self.get_clock().now().nanoseconds
        self.speed_source = "GROUND_TRUTH"

    def path_callback(self, msg: WaypointArrayStamped):
        # dt for PID
        now_ns = self.get_clock().now().nanoseconds
        now = now_ns * 1e-9
        dt  = max(1e-6, now - self.prev_time)
        self.prev_time = now

        # Extract [x,y]
        wps = [[wp.position.x, wp.position.y] for wp in msg.waypoints]
        if not wps:
            return

        # Static look‐ahead waypoint
        idx = self.get_parameter("static_lookahead_idx").value
        look_wp = wps[idx] if idx < len(wps) else wps[-1]

        # Compute raw steering angle (deg & rad)
        raw_deg   = math.degrees(math.atan2(look_wp[1], look_wp[0]))
        abs_deg   = abs(raw_deg)

        # Enforce ±steer_limit: if exceeded, find first wp with |angle|≈steer_cap
        if abs_deg > self._steer_lim:
            for wp in wps:
                deg_i = abs(math.degrees(math.atan2(wp[1], wp[0])))
                if deg_i >= self._steer_cap:
                    raw_deg = math.copysign(self._steer_cap, raw_deg)
                    break

        self.steer_rad = math.radians(raw_deg)

        # PID on steering magnitude (target 0°) → pid_out∈[0,1]
        pid_out = self.steer_pid.update(abs(raw_deg), dt)
        self.last_pid_out = float(pid_out)

        # Map pid_out to accel in [–2, +1]: 0→+1, 1→–2
        accel_cmd = 1.0 - 3.0 * pid_out
        accel_cmd = max(-1.0, min(1.0, accel_cmd))

        #Enforce speed window [min, max]
        speed_fresh = (now_ns - self.last_speed_update_ns) <= int(float(self.speed_timeout_sec) * 1e9)
        if speed_fresh:
            if self.speed < self.min_speed:
                accel_cmd = +1.0
            elif self.speed > self.max_speed:
                accel_cmd = -1.0
        else:
            accel_cmd = min(accel_cmd, 0.0)
        if self.stop_triggered:
            self.get_logger().info(" Stopping in progress: Ignoring path control")
            self.publish_command(-1.0, 0.0)  # Maintain braking
            return

        # Publish drive & viz
        self.publish_command(accel_cmd, float(self.steer_rad))
        self.publish_visualisation(accel_cmd, raw_deg,self.speed)
        self.publish_static_lookahead_marker(look_wp)
    def can_state_callback(self, msg):
        # self.get_logger().info("---------->  self.as_state:" + str(msg.as_state))
        # self.get_logger().info("---------->  self.ami_state:" + str(msg.ami_state))
        self.get_logger().info(
            " ami_state :" + str(msg.ami_state) + " self.state :" + str(msg.as_state),
            throttle_duration_sec=1.0,
        )
        self.ws_ami_state = msg.ami_state
        self.ws_as_state = msg.as_state
    def wheel_speed_callback(self, msg):
        self.ws_can_steering= msg.steering
        rear_rpm = 0.5 * (abs(msg.lb_speed) + abs(msg.rb_speed))
        wheel_circumference = 2.0 * math.pi * float(self.wheel_radius_m)
        self.speed = (rear_rpm / 60.0) * wheel_circumference
        self.last_speed_update_ns = self.get_clock().now().nanoseconds
        self.speed_source = "WHEEL_RPM"

    def turn_memory_callback(self, msg: Bool):
        self.turn_memory_active = bool(msg.data)



    def consig_callback(self,msg):
        self.get_logger().info("Control signal : " + str(msg.data))
        
        if msg.data == 10:
            acceleration_cmd = -5.0
            self.steer_change_cmd = 0.5
            # : Need to change this afterwards
            #steering_cmd = self.prev_steering
            steering_cmd = self.steer_rad
            self.publish_command(acceleration_cmd, steering_cmd)
            # rclpy.time.sleep(5)
            
            msg = Bool()
            msg.data = True  # or 1
            self.get_logger().info("---------->  ENGAGING STOP In Acceleration")
            # self.mission_flag_pub.publish(msg)
            self.mission_completed_pub.publish(msg)
            #self.mission_flag_pub.publish(msg)
        elif msg.data == 30:
            acceleration_cmd = -1.0
            # : Need to change this afterwards
            steering_cmd = self.steer_rad
            self.publish_command(acceleration_cmd, steering_cmd)
        elif msg.data == 40:
            acceleration_cmd = -1
            #  : Need to change this afterwards
            steering_cmd = self.steer_rad
            self.publish_command(acceleration_cmd, steering_cmd)
        elif msg.data == 90:
            acceleration_cmd = -1
            # : Need to change this afterwards
            steering_cmd = self.steer_rad
            self.publish_command(acceleration_cmd, steering_cmd)



    def cones_callback(self, msg: ConeArrayWithCovariance):
        blue_cones = self.convert(msg.blue_cones)
        yellow_cones = self.convert(msg.yellow_cones)
        orange_cones = self.convert(msg.orange_cones)
        big_orange_cones = self.convert(msg.big_orange_cones)

        self.get_logger().info(f"Number of small orange cones: {len(orange_cones)}")

        # Check stopping condition
        if len(blue_cones) == 0 and len(yellow_cones) == 0 and len(big_orange_cones) == 0 and len(orange_cones) >= 4:
            if self.stop_triggered:
                self.get_logger().info("Stopping already triggered, ignoring cones")
                return
            self.stop_triggered = True
            self.get_logger().warn("Stop condition detected from cones, engaging braking")
            self.publish_command(-2.0, 0.0)
            stop_msg = Bool()
            stop_msg.data = True
            self.mission_completed_pub.publish(stop_msg)
            self.driving_flag_pub.publish(stop_msg)
            
    def convert(self, cones):
        """
        Converts a cone array message into a np array of complex
        """
        return np.array([c.point.x + 1j * c.point.y for c in cones])


    def publish_command(self, acceleration: float, steering: float):
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.header.frame_id      = "pure_pursuit"
        msg.drive.steering_angle = steering
        msg.drive.acceleration   = acceleration
        self.cmd_pub.publish(msg)

    def publish_visualisation(self, accel: float, steer_deg: float, speed: float):
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "base_footprint"
        m.type            = Marker.TEXT_VIEW_FACING
        m.action          = Marker.ADD
        m.ns              = "controls"
        m.id              = 0
        m.scale.x = 0.0
        m.scale.y = 0.0
        m.scale.z = 0.5
        m.pose.position.x = 3.0
        m.pose.position.y = 4.0
        m.pose.position.z = 1.0
        m.pose.orientation.w = 1.0
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.text = (
            f"Speed: {speed:.2f}  Range:[{self.min_speed:.1f},{self.max_speed:.1f}]\n"
            f"Steer: {steer_deg:.6f}°\n"
            f"Accel: {accel:.2f}\n"
            f"Memory: {'ACTIVE' if self.turn_memory_active else 'OFF'}\n"
            #f"SpeedSrc: {self.speed_source}\n"
            f"PID_out: {self.last_pid_out:.6f}"
        )
        self.viz_pub.publish(m)

    def publish_static_lookahead_marker(self, wp):
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "base_footprint"
        m.ns              = "static_lookahead"
        m.id              = 42
        m.type            = Marker.SPHERE
        m.action          = Marker.ADD
        m.pose.position.x = wp[0]
        m.pose.position.y = wp[1]
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.r = 1.0
        m.color.a = 1.0
        self.index_viz_pub.publish(m)

def main():
    rclpy.init()
    node = Control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
