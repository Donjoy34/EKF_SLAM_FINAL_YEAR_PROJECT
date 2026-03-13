from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wheel_topic_is_stamped = LaunchConfiguration('wheel_topic_is_stamped')
    map_file_path = LaunchConfiguration('map_file_path')
    publish_tf = LaunchConfiguration('publish_tf')
    occupancy_resolution_m = LaunchConfiguration('occupancy_resolution_m')
    occupancy_width_cells = LaunchConfiguration('occupancy_width_cells')
    occupancy_height_cells = LaunchConfiguration('occupancy_height_cells')
    enable_loop_closure = LaunchConfiguration('enable_loop_closure')
    loop_search_radius_m = LaunchConfiguration('loop_search_radius_m')
    loop_yaw_gate_deg = LaunchConfiguration('loop_yaw_gate_deg')
    loop_min_landmark_overlap = LaunchConfiguration('loop_min_landmark_overlap')

    return LaunchDescription([
        DeclareLaunchArgument('wheel_topic_is_stamped', default_value='true'),
        DeclareLaunchArgument('map_file_path', default_value=''),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('occupancy_resolution_m', default_value='0.25'),
        DeclareLaunchArgument('occupancy_width_cells', default_value='480'),
        DeclareLaunchArgument('occupancy_height_cells', default_value='480'),
        DeclareLaunchArgument('enable_loop_closure', default_value='true'),
        DeclareLaunchArgument('loop_search_radius_m', default_value='3.5'),
        DeclareLaunchArgument('loop_yaw_gate_deg', default_value='35.0'),
        DeclareLaunchArgument('loop_min_landmark_overlap', default_value='3'),
        Node(
            package='localization',
            executable='slam',
            name='slam',
            output='screen',
            parameters=[
                {'wheel_topic_is_stamped': wheel_topic_is_stamped},
                {'map_file_path': map_file_path},
                {'publish_tf': publish_tf},
                {'occupancy_resolution_m': occupancy_resolution_m},
                {'occupancy_width_cells': occupancy_width_cells},
                {'occupancy_height_cells': occupancy_height_cells},
                {'enable_loop_closure': enable_loop_closure},
                {'loop_search_radius_m': loop_search_radius_m},
                {'loop_yaw_gate_deg': loop_yaw_gate_deg},
                {'loop_min_landmark_overlap': loop_min_landmark_overlap},
            ],
        )
    ])
