#!/usr/bin/python3
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import cv_bridge
from message_filters import Subscriber
from message_filters import ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from eufs_msgs.msg import BoundingBoxes


class DepthViewer(Node):
    def __init__(self):
        super().__init__('depth_viewer')
   # Subscribe to the left rectified image
        self.left_sub_ = Subscriber(
            self,
            Image,
            '/zed2i/zed_node/left/image_rect_color',
        )

        # Subscribe to the right rectified image
        self.right_sub_ = Subscriber(
            self,
            Image,
            '/zed2i/zed_node/right/image_rect_color',
        )

        # Create the approximate time synchroniser to sync the subscribers
        self.ats_ = ApproximateTimeSynchronizer(
            [self.left_sub_, self.right_sub_],
            100,
            0.1  # The slop should almost always be small!
        )
        # Register the callback with the approximate time synchroniser
        self.ats_.registerCallback(self.callback)

    def callback(self, left_img_msg, right_img_msg):
        """
        Match features between left & right images and calculate a depth map.
        """
        print("Received data from cameras successfully.")
        pass


def main():
    rclpy.init(args=sys.argv)
    dv = DepthViewer()
    rclpy.spin(dv)
    rclpy.shutdown()
