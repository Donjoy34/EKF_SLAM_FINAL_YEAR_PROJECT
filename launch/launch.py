from os.path import join
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(
        #         join('src', 'eufs_sim', 'eufs_launcher', 'launch', 'simulation.launch.py')
        #     )
        # ),

        Node(
            package='localization',
            executable='sensor_fusion',
            name='sensor_fusion'
        ),
        
        # Node(
        #     package='plotter',
        #     executable='plotter',
        #     name='plotter'
        # )
    ])
