import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'
        ),
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']
        ),
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='listener',
            output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'listener']
        ),
    ])
