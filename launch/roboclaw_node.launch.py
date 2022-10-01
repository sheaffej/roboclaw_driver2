from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node_name = DeclareLaunchArgument('name', default_value='roboclaw')
    dev_names = DeclareLaunchArgument('dev_names', default_value='/dev/ttyACM0')
    baud = DeclareLaunchArgument('baud', default_value='115200')
    address = DeclareLaunchArgument('address', default_value='128')
    loop_hz = DeclareLaunchArgument('loop_hz', default_value='10')
    deadman_secs = DeclareLaunchArgument('deadman_secs', default_value='3')
    speed_cmd_topic = DeclareLaunchArgument('speed_cmd_topic', default_value='speed_command')
    stats_topic = DeclareLaunchArgument('stats_topic', default_value='stats')
    test_mode = DeclareLaunchArgument('test_mode', default_value='false')

    roboclaw_node = Node(
        package='roboclaw_driver',
        executable='roboclaw_node',
        name=LaunchConfiguration('name'),
        parameters=[
            {'dev_names': LaunchConfiguration('dev_names')},
            {'baud_rate': LaunchConfiguration('baud')},
            {'address': LaunchConfiguration('address')},
            {'loop_hz': LaunchConfiguration('loop_hz')},
            {'deadman_secs': LaunchConfiguration('deadman_secs')},
            {'speed_cmd_topic': LaunchConfiguration('speed_cmd_topic')},
            {'stats_topic': LaunchConfiguration('stats_topic')},
            {'test_mode': LaunchConfiguration('test_mode')}
        ]
    )

    return LaunchDescription(
        [
            # Arguments
            node_name, dev_names, baud, address, loop_hz, deadman_secs,
            speed_cmd_topic, stats_topic, test_mode,

            # Node
            roboclaw_node
        ]
    )
