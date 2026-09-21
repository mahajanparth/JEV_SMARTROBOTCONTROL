from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default = os.path.join(get_package_share_directory('jev_localization_recovery'),
                           'config', 'recovery_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_recovery', default_value='false'),
        DeclareLaunchArgument('selector_provider', default_value='mock'),
        Node(package='jev_localization_recovery', executable='recovery_node',
             name='jev_recovery', output='screen', parameters=[
                 LaunchConfiguration('params_file'),
                 {'use_sim_time': LaunchConfiguration('use_sim_time'),
                  'enable_recovery': LaunchConfiguration('enable_recovery'),
                  'selector_provider': LaunchConfiguration('selector_provider')}]),
    ])
