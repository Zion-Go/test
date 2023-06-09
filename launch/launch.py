import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='stereocam_publisher',
            executable='leftcamera_publisher',
            name='leftcamera'),
        launch_ros.actions.Node(
            package='stereocam_publisher',
            executable='rightcamera_publisher',
            name='rightcamera'),
  ])