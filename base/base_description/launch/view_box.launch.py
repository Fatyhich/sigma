import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('base_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'test_basement.urdf.xacro'])

    # Declare launch argument for scale
    scale_arg = DeclareLaunchArgument(
        'scale',
        default_value='0.01 0.01 0.01',
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([scale_arg, rsp, rviz])