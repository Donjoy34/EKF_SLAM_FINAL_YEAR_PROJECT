from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    static_lookahead_idx = LaunchConfiguration('static_lookahead_idx')
    min_speed = LaunchConfiguration('min_speed')
    max_speed = LaunchConfiguration('max_speed')
    kp_acc = LaunchConfiguration('Kp_acc')
    ki_acc = LaunchConfiguration('Ki_acc')
    kd_acc = LaunchConfiguration('Kd_acc')
    kp_steer = LaunchConfiguration('Kp_steer')
    ki_steer = LaunchConfiguration('Ki_steer')
    kd_steer = LaunchConfiguration('Kd_steer')
    steer_sensor_in_rad = LaunchConfiguration('steer_sensor_in_rad')
    steer_limit_deg = LaunchConfiguration('steer_limit_deg')
    steer_cap_deg = LaunchConfiguration('steer_cap_deg')

    node = Node(
        package="control",
        executable="control_node",
        parameters=[
            {"static_lookahead_idx": static_lookahead_idx},
            {"min_speed": min_speed},
            {"max_speed": max_speed},
            {"Kp_acc": kp_acc},
            {"Ki_acc": ki_acc},
            {"Kd_acc": kd_acc},
            {"Kp_steer": kp_steer},
            {"Ki_steer": ki_steer},
            {"Kd_steer": kd_steer},
            {"steer_sensor_in_rad": steer_sensor_in_rad},
            {"steer_limit_deg": steer_limit_deg},
            {"steer_cap_deg": steer_cap_deg},
        ]
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('static_lookahead_idx', default_value='6'))
    ld.add_action(DeclareLaunchArgument('min_speed', default_value='0.5'))
    ld.add_action(DeclareLaunchArgument('max_speed', default_value='1.0'))
    ld.add_action(DeclareLaunchArgument('Kp_acc', default_value='0.8'))
    ld.add_action(DeclareLaunchArgument('Ki_acc', default_value='0.01'))
    ld.add_action(DeclareLaunchArgument('Kd_acc', default_value='0.2'))
    ld.add_action(DeclareLaunchArgument('Kp_steer', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('Ki_steer', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('Kd_steer', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('steer_sensor_in_rad', default_value='True'))
    ld.add_action(DeclareLaunchArgument('steer_limit_deg', default_value='60.0'))
    ld.add_action(DeclareLaunchArgument('steer_cap_deg', default_value='59.0'))
    ld.add_action(node)
    return ld
