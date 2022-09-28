from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    node_name = DeclareLaunchArgument('name', default_value='roboclaw')
    test_mode = DeclareLaunchArgument('test_mode', default_value=False)
    dev_names = DeclareLaunchArgument('dev_names', default_value='/dev/ttyACM0')
    baud = DeclareLaunchArgument('baud', default_value=115200)
    address = DeclareLaunchArgument('address', default_value=128)
    loop_hz = DeclareLaunchArgument('loop_hz', default_value=10)
    deadman_secs = DeclareLaunchArgument('deadman_secs', default_value=3)
    speed_cmd_topic = DeclareLaunchArgument('speed_cmd_topic', default_value=f"{node_name.default_value}/speed_command")
    stats_topic = DeclareLaunchArgument('stats_topic', default_value=f"{node_name.default_value}/stats")



    TODO: Read this: https://docs.ros.org/en/galactic/Tutorials/Intermediate/Launch/Using-Substitutions.html

    
    roboclaw_node = Node(
        package='roboclaw_driver',
        # namespace=turtlesim_ns,
        executable='roboclaw_node.py',
        name=node_name,
        parameters=
    )
