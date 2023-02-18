from rclpy import init, spin, shutdown
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, ApproximateTimeSynchronizer
from numpy import array, eye
from numpy.linalg import inv
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped

class SensorFusion(Node):

    def __init__(self):

        super().__init__('sensor_fusion')

        # Initialise subscribers and time synchronizer
        ApproximateTimeSynchronizer([Subscriber(self, TwistWithCovarianceStamped, 'ros_can/twist'),
                                     Subscriber(self, Imu, 'imu', qos_profile=qos_profile_sensor_data)],
                                     10, 1).registerCallback(self.callback)

        # # Create subscribers
        # self.cones_sub = self.create_subscription(Imu, "/imu/data", self.imu_callback, 1)

        # Initialise velocity publisher
        self.publisher = self.create_publisher(TwistWithCovarianceStamped, 'velocity', 1)

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

    def callback(self, wheel_data: TwistWithCovarianceStamped, imu_data: Imu):
        self.get_logger().info("Imu data ------------------------------------------------>    :")
        self.get_logger().info(str(wheel_data))

        self.get_logger().info('Entered callback')

        self.publisher .publish(wheel_data)



        # Imu data publisher command :
        # ros2 topic pub /imu sensor_msgs/Imu '{header: {stamp: {sec: 732, nanosec: 55000000}, frame_id: "imu_frame"}, linear_acceleration: {x: 1.0, y: 2.0, z: 3.0}, angular_velocity: {x: 4.0, y: 5.0, z: 6.0}}'

        # wheel encoder data publisher command:
        # ros2 topic pub /ros_can/twist geometry_msgs/msg/TwistWithCovarianceStamped '{header: {stamp: {sec: 732, nanosec: 55000000}, frame_id: "base_link"}, twist: {twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}, covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,0.0, 0.0, 0.0, 0.1]}}'

       


def main():
    init()
    spin(SensorFusion())
    shutdown()

if __name__ == '__main__':
    main()