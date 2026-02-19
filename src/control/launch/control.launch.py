from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package="control",
        executable="control_node",
        parameters=[
            {"look_ahead": 4.0},
            {"L": 1.5},         # wheel base
            {"K_p": 1.0},
            {"K_i": 1.0},
            {"K_d": 1.0},
            {"max_lat_acc": 5.0},
            {"safe_speed": 1.5},
            {"max_speed": 4.5},
            {"buffer_len": 30}      # length of the error buffer
        ]
    )

    ld = LaunchDescription()
    ld.add_action(node)
    return ld
