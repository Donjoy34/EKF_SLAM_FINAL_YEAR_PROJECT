import rclpy
# from rclpy import init, spin, shutdown
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.time import Time
from rclpy.clock import ClockType
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, ApproximateTimeSynchronizer
from numpy import array, eye
from numpy.linalg import inv
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


class SensorFusion(Node):

    def __init__(self, name):

        super().__init__(name)

        # Initialise subscribers and time synchronizer
        ApproximateTimeSynchronizer([Subscriber(self, Odometry, '/ground_truth/odom'),
                                     Subscriber(self, Imu, 'imu/data', qos_profile=qos_profile_sensor_data)],
                                     10, 1).registerCallback(self.callback)

        # # Create subscribers
        # self.cones_sub = self.create_subscription(Imu, "/imu/data", self.imu_callback, 1)

        # Initialise velocity publisher
        self.publisher = self.create_publisher(Odometry, 'velocity', 1)


        self.Odom_publisher = self.create_publisher(Odometry, 'Odom_Velocity', 1)
        self.Imu_publisher = self.create_publisher(Imu, 'Imu_Velocity', 1)

        

        self.previous_time = self.get_clock().now()

        # Initial state vector [velocity, acceleration]
        self.state = array([0.0, 0.0])

        # Covariance matrix for the uncertainty in the initial state
        self.covariance = array([[0.0, 0.0],
                                 [0.0, 0.0]])

        #TODO figure out how to compute this (hint: look at robot_localization)
        self.process_noise_covariance = array([[1.0, 0.0],
                                               [0.0, 1.0]])

        self.get_logger().info('Initialised sensor fusion node')
           

    def state_transition_matrix(self, dt):
        return

    def wheel_prediction_matrix(self):
        return

    def imu_prediction_matrix(self):
        return

    def wheel_noise_covariance(self, wheel_data: TwistWithCovarianceStamped):
        return

    def imu_noise_covariance(self, imu_data: Imu):
        return

    def callback(self, wheel_data: Odometry, imu_data: Imu):
        # self.get_logger().info("Imu data : imu_data.linear_acceleration.x ------------------------------------------------>    :")
        # self.get_logger().info(str(imu_data.linear_acceleration.x))

        # ---------------- Test contents to be removed : From here --------------------->

        time_delta = 2
        current_time = self.get_clock().now()
        new_state=[0.0, 0.0]
        linear_acceleration = imu_data.linear_acceleration.x

        # Add 2 seconds to previous_time
        new_time_sec = self.previous_time.nanoseconds // 1e9 + time_delta
        new_time_nsec = self.previous_time.nanoseconds % 1e9
        new_time = Time(seconds=new_time_sec, nanoseconds=new_time_nsec, clock_type=ClockType.ROS_TIME)

        # self.get_logger().info("previous_time:   :")
        # self.get_logger().info(str(self.previous_time))

        # self.get_logger().info("new_time:   :")
        # self.get_logger().info(str(new_time))

        # self.get_logger().info("current_time:   :")
        # self.get_logger().info(str(current_time))

        if new_time < current_time:
            new_state[0]= self.state[0] + ( linear_acceleration * time_delta )  # Velocity
            new_state[1]= linear_acceleration                                   # Acceleration

            self.state = new_state

            self.previous_time = new_time

            self.get_logger().info("UPDATED STATE :   :")
            self.get_logger().info(str(self.state))

        # self.get_logger().info("self.previous_time    :")
        # self.get_logger().info(str(self.previous_time))

        # self.get_logger().info("Current_time :   :")
        # self.get_logger().info(str(self.get_clock().now()))

        # self.state

        # self.get_logger().info("new_state:   :")
        # self.get_logger().info(str(new_state))


        # self.state.append(new_state)

        # numpy.append(self.state,new_state)

        # self.get_logger().info("Updated state:   :")
        # self.get_logger().info(str(self.state))

        # <-------------------------- Untill here  <-------------------------

        # self.get_logger().info('Entered callback')

        self.publisher.publish(wheel_data)

        imu_data.linear_acceleration.y = self.state[0]

        self.Odom_publisher.publish(wheel_data)
        self.Imu_publisher.publish(imu_data)

        # Imu data publisher command :
        # ros2 topic pub /imu sensor_msgs/Imu '{header: {stamp: {sec: 732, nanosec: 55000000}, frame_id: "imu_frame"}, linear_acceleration: {x: 1.0, y: 2.0, z: 3.0}, angular_velocity: {x: 4.0, y: 5.0, z: 6.0}}'

        # wheel encoder data publisher command:
        # ros2 topic pub /ros_can/twist geometry_msgs/msg/TwistWithCovarianceStamped '{header: {stamp: {sec: 732, nanosec: 55000000}, frame_id: "base_link"}, twist: {twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}, covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,0.0, 0.0, 0.0, 0.1]}}'

       


def main():
    # init()
    # spin(SensorFusion())
    # shutdown()
    rclpy.init(args=None)
    node = SensorFusion("SensorFusion")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()