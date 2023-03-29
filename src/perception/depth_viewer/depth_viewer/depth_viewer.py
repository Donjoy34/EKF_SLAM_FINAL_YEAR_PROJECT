#!/usr/bin/python3
import sys
import cv2
from cv2 import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import cv_bridge
from message_filters import Subscriber
from message_filters import TimeSynchronizer
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from eufs_msgs.msg import BoundingBoxes


class DepthViewer(Node):
    def __init__(self):
        super().__init__('depth_viewer')


def main():
    rclpy.init(args=sys.argv)
    dv = DepthViewer()
    rclpy.spin(dv)
    rclpy.shutdown()
