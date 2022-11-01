from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package="planning",
        executable="planning_node",
        parameters=[
            {"threshold": 6.0},   # distance threshold for connecting blue and yellow cones
        ]
    )

    ld = LaunchDescription()
    ld.add_action(node)
    return ld
