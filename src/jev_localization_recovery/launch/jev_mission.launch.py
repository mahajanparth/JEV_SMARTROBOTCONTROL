"""Random Jev mission with unknown initial pose, Nav2 and local dashboard."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('jev_localization_recovery')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gazebo = get_package_share_directory('gazebo_ros')
    with open(os.path.join(tb3, 'urdf', 'turtlebot3_burger.urdf')) as source:
        description = source.read()
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('enable_recovery', default_value='true'),
        DeclareLaunchArgument('selector_provider', default_value='jev'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('autostart', default_value='false'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(tb3, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gzserver.launch.py')),
                                 launch_arguments={'world': os.path.join(share, 'worlds', 'jev_mission.world')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gzclient.launch.py')),
                                 condition=IfCondition(gui)),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': description}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'turtlebot3_burger', '-file', os.path.join(tb3, 'models', 'turtlebot3_burger', 'model.sdf'),
                        '-x', '3.8', '-y', '2.7', '-z', '0.01']),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'use_sim_time': True, 'yaml_filename': os.path.join(share, 'maps', 'jev_house.yaml')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[os.path.join(share, 'config', 'house_amcl.yaml'), {'set_initial_pose': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='localization_manager',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        Node(package='jev_localization_recovery', executable='mission_bridge',
             parameters=[os.path.join(share, 'config', 'obstacle_safety.yaml'), {'use_sim_time': True}], output='screen'),
        Node(package='jev_localization_recovery', executable='mission_node', name='jev_recovery', output='screen',
             parameters=[os.path.join(share, 'config', 'recovery_params.yaml'),
                         {'use_sim_time': True, 'enable_recovery': LaunchConfiguration('enable_recovery'),
                          'selector_provider': LaunchConfiguration('selector_provider'),
                          'base_frame': 'base_footprint', 'cmd_vel_topic': '/recovery/cmd_vel',
                          'action_timeout': 70.0, 'jev_timeout': 5.0, 'decision_timeout': 6.0,
                          'seed': LaunchConfiguration('seed'), 'autostart': LaunchConfiguration('autostart')}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[os.path.join(share, 'config', 'mission_nav2.yaml')]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=[os.path.join(share, 'config', 'mission_nav2.yaml')],
             remappings=[('cmd_vel','/navigation/cmd_vel')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[os.path.join(share, 'config', 'mission_nav2.yaml'),
                         {'default_nav_to_pose_bt_xml': os.path.join(share,'config','mission_nav.xml'),
                          'default_nav_through_poses_bt_xml': os.path.join(share,'config','mission_through.xml')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='navigation_manager',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['planner_server','controller_server','bt_navigator']}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
             arguments=['-d', os.path.join(share, 'config', 'mission.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
