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

        self.get_logger().info('Entered callback')
        


def main():
    init()
    spin(SensorFusion())
    shutdown()

if __name__ == '__main__':
    main()