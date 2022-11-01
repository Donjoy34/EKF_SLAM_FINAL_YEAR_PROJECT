from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("planning"),
                                       "/launch/planning.launch.py"])
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("control"),
                                       "/launch/control.launch.py"])
    )

    ld = LaunchDescription()

    ld.add_action(planner)
    ld.add_action(controller)

    return ld