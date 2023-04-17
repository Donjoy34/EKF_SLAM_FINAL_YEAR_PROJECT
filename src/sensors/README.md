This Repo contains the ROS2 packages for Sensors  


## Qcar

RPLIDAR A2 :  https://github.com/slamtec/rplidar_ros/tree/ros2
> - Clone rplidar_ros2 package from github : `git clone -b ros2 https://github.com/slamtec/rplidar_ros.git`
> - package_insallation : `sudo apt install ros-galactic-rplidar-ros`
> - Go to Repository : `cd rplidar_ros/`
> - Build rplidar_ros2 package : `colcon build --symlink-install`
> - Set up the USB port : `sudo chmod 666 /dev/ttyUSB0`
> - Addint user to dailout : `sudo adduser $USER dialout`
> - Package environment setup : `source ./install/setup.bash`
> - launch : `ros2 launch rplidar_ros2 view_rplidar_launch.py`
> - Tweeks : change the Reliability Policy of LaserScan from "System Default" to "Best Effort"

Intel realsence D450: https://github.com/IntelRealSense/realsense-ros


## ADS-DV

Zed 2i Camera: https://github.com/stereolabs/zed-ros2-wrapper

Velodyne Vlp 16 PUCK:

## References

Qcar : https://www.quanser.com/products/qcar/
