import os
import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    base_desc_pkg = get_package_share_directory('base_description')
    orbbec_pkg = get_package_share_directory('orbbec_camera')
    ouster_pkg = get_package_share_directory('ouster_ros')

    available_rviz_configs = [f for f in os.listdir(os.path.join(base_desc_pkg, 'rviz')) if f.endswith('.rviz')]

    params = dict([aa for aa in [aa.split(':=') for aa in sys.argv] if len(aa) == 2])
    if ('rviz_config' not in params or params['rviz_config'] not in available_rviz_configs):
        print('USAGE:')
        print('ros2 launch base_description full_system.launch.py [rviz_config:=<config>]')
        print('\nAvailable arguments for <config> (optional):')
        print('\n'.join(available_rviz_configs))

    # URDF/Robot description
    xacro_file = PathJoinSubstitution([base_desc_pkg, 'urdf', 'test_platform.urdf.xacro'])

    scale_arg = DeclareLaunchArgument(
        'scale',
        default_value='1 1 1',
        description='Scale for the mesh (x y z)'
    )

    scale = LaunchConfiguration('scale')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' scale:="', scale, '"']),
            value_type=str
        )
    }

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Configure RViz with config file if specified
    rviz_args = []
    if 'rviz_config' in params and params['rviz_config'] in available_rviz_configs:
        rviz_config_file = PathJoinSubstitution([base_desc_pkg, 'rviz', params['rviz_config']])
        rviz_args = ['-d', rviz_config_file]

    # RViz (only one instance)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=rviz_args,
        output='screen'
    )

    # Orbbec Camera Launch
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_pkg, 'launch', 'orbbec_camera.launch.py')
        ),
        launch_arguments={
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'true'
        }.items()
    )

    # Ouster LiDAR Launch (without viz)
    ouster_params_file = os.path.join(ouster_pkg, 'config','beluga_driver_params.yaml')

    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ouster_pkg, 'launch', 'driver.launch.py')
        ),
        launch_arguments={
            'params_file': ouster_params_file,
            'viz': 'false'
        }.items()
    )

    # Static TF: lidar_link -> os_sensor
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.02', '0', '0', '0', 'lidar_link', 'os_sensor'],
        output='screen'
    )

    return LaunchDescription([
        scale_arg,
        robot_state_publisher,
        rviz,
        orbbec_launch,
        ouster_launch,
        static_tf
    ])
