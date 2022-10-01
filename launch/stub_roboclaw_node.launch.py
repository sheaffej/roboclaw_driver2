from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    loop_hz = DeclareLaunchArgument('loop_hz', default_value='10')
    speed_cmd_topic = DeclareLaunchArgument('speed_cmd_topic', default_value='speed_command')
    stats_topic = DeclareLaunchArgument('stats_topic', default_value='stats')

    return LaunchDescription(
        [
            # Args
            loop_hz, speed_cmd_topic, stats_topic,

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare('roboclaw_node'),
                                'launch',
                                'roboclaw_node.launch.py'
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    'name': 'roboclaw',
                    'dev_names': '/dev/ttyACM0',
                    'baud': '115200',
                    'address': '128',
                    'loop_hz': LaunchConfiguration('loop_hz'),
                    'deadman_secs': '3',
                    'speed_cmd_topic': LaunchConfiguration('speed_cmd_topic'),
                    'stats_topic': LaunchConfiguration('stats_topic'),
                    'test_mode': 'true'
                }.items()
            )
        ]
    )
