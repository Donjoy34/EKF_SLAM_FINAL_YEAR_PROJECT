from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_odom_topic = LaunchConfiguration('slam_odom_topic')
    gt_odom_topic = LaunchConfiguration('gt_odom_topic')
    landmark_topic = LaunchConfiguration('landmark_topic')
    output_dir = LaunchConfiguration('output_dir')
    run_name = LaunchConfiguration('run_name')
    auto_export_on_shutdown = LaunchConfiguration('auto_export_on_shutdown')

    return LaunchDescription([
        DeclareLaunchArgument('slam_odom_topic', default_value='/slam/odom'),
        DeclareLaunchArgument('gt_odom_topic', default_value='/ground_truth/odom'),
        DeclareLaunchArgument('landmark_topic', default_value='/slam/landmarks'),
        DeclareLaunchArgument('output_dir', default_value='~/uh-fs-ai/slam_eval'),
        DeclareLaunchArgument('run_name', default_value='ekf_slam_run'),
        DeclareLaunchArgument('auto_export_on_shutdown', default_value='true'),
        Node(
            package='localization',
            executable='slam_evaluator',
            name='slam_evaluator',
            output='screen',
            parameters=[
                {'slam_odom_topic': slam_odom_topic},
                {'gt_odom_topic': gt_odom_topic},
                {'landmark_topic': landmark_topic},
                {'output_dir': output_dir},
                {'run_name': run_name},
                {'auto_export_on_shutdown': auto_export_on_shutdown},
            ],
        )
    ])
