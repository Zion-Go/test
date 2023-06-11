
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory('stereocam_publisher'),
                          'config',
                          'nodes_config.yaml'
    )
    lcam_node = Node(
        package = "stereocam_publisher",
        name = "leftcamera_publisher",
        executable = "leftcamera_publisher",
        parameters = [
            {'serial': "45020167"}
        ]
    )
    rcam_node = Node(
        package = "stereocam_publisher",
        name = "rightcamera_publisher",
        executable = "rightcamera_publisher",
        parameters = [
            {'serial': "45020169"}
        ]
    )
    ld.add_action(lcam_node)
    ld.add_action(rcam_node)
    return ld
