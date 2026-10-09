import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 场景文件随 dog_cpp 包一起安装到 share/dog_cpp/scenes/ 下，
    # 因此这里用 get_package_share_directory 定位，仓库 clone 到任何路径都能跑。
    scene = os.path.join(
        get_package_share_directory('dog_cpp'), 'scenes', 'flat_scene.xml')

    sim = Node(
        package='dog_cpp',
        executable='sim',
        name='sim_node',
        output='screen',
        arguments=[scene],          # sim 可执行文件用 argv[1] 接收场景路径
    )

    controller = Node(
        package='dog_cpp',
        executable='controller',
        name='controller_node',
        output='screen',
    )

    teleop = Node(
        package='dog_teleop',
        executable='teleop',
        name='teleop_node',
        output='screen',
    )

    return LaunchDescription([sim, controller, teleop])
