from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(package = 'turtlebot4_custom',
                 executable = 'behavior_decision_server',
                 output = 'screen',
            ),
            Node(package = 'turtlebot4_custom',
                 executable = 'object_detector',
                 output = 'screen'
            ),
            Node(package = 'turtlebot4_custom',
                 executable= 'lane_detector',
                 output = 'screen',
            ),
        ]
    )