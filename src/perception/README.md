
Launch the Eufs_launcher on a new terminal and run the below commands on separate terminals after sourcing the appropriate setup files.

`ros2 launch eufs_launcher eufs_launcher.launch.py`


Launch the depth_viewer node:

`ros2 run depth_viewer depth_viewer`


Launch Rviz2 Visualizer:

`rviz2 -d src/perception/rviz2/perception_workshop.rviz`