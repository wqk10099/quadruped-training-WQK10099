import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    scene = os.path.join(
        get_package_share_directory("dog_cpp"),
        "scenes",
        "flat_scene.xml",
    )

    sim = Node(
        package="dog_cpp",
        executable="sim",
        name="sim_node",
        output="screen",
        arguments=[scene],
    )

    rl_controller = Node(
        package="dog_cpp",
        executable="rl_controller",
        name="rl_controller",
        output="screen",
        parameters=[
            {"enable_publish": True},
        ],
    )

    return LaunchDescription([
        sim,
        rl_controller,
    ])