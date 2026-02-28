from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package="control",
        executable="control_node",
        parameters=[
            {"static_lookahead_idx": 6},
            {"min_speed": 0.5},
            {"max_speed": 1.0},
            {"Kp_acc": 0.0},
            {"Ki_acc": 0.0},
            {"Kd_acc": 0.0},
            {"steer_limit_deg": 60.0},
            {"steer_cap_deg": 59.0},
        ]
    )

    ld = LaunchDescription()
    ld.add_action(node)
    return ld
