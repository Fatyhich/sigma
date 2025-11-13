import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('base_description')
    available_urdf_files = [f for f in os.listdir(os.path.join(pkg_share, 'urdf')) if f.startswith('test_')]
    available_rviz_configs = [f for f in os.listdir(os.path.join(pkg_share, 'rviz')) if f.endswith('.rviz')]

    params = dict([aa for aa in [aa.split(':=') for aa in sys.argv] if len(aa) == 2])
    if ('model' not in params or params['model'] not in available_urdf_files):
        print('USAGE:')
        print('ros2 launch base_description view_box.launch.py model:=<model> [rviz_config:=<config>]')
        print('Available arguments for <model> are as follows:')
        print('\n'.join(available_urdf_files))
        print('\nAvailable arguments for <config> (optional):')
        print('\n'.join(available_rviz_configs))
        return LaunchDescription()

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', params['model']])

    # Declare launch argument for scale
    scale_arg = DeclareLaunchArgument(
        'scale',
        default_value='1 1 1',
        description='Scale for the mesh (x y z)'
    )

    # Get the scale value
    scale = LaunchConfiguration('scale')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' scale:="', scale, '"']),
            value_type=str
        )
    }

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Configure RViz with config file if specified
    rviz_args = []
    if 'rviz_config' in params and params['rviz_config'] in available_rviz_configs:
        rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', params['rviz_config']])
    else:
        rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'default_config.rviz'])
    rviz_args = ['-d', rviz_config_file]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=rviz_args,
        output='screen'
    )

    return LaunchDescription([scale_arg, rsp, rviz])