import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(os.getcwd(),
      '../config',
      'controller_config.yaml'
      )

    return LaunchDescription(
        [
            Node(
                package="move_to_pose_ros",
                executable="controller",
                name="pose_controller",
                parameters=[config],
                output="screen",
            ),
        ]
    )
