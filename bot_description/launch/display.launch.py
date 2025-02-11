from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch
import launch_ros
import xacro
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    share_dir = get_package_share_directory('bot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'omnidrive.xacro')
    # map_file = os.path.join(share_dir, 'config', 'new_map.yaml')
    # param_file = os.path.join(share_dir, 'config', 'navigation.yaml')
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    # nav2_launch_path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    # Process the xacro file to get the robot description in XML format
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Declare Launch Arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch Configurations
    show_gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Define the nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': use_sim_time}
        ]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'yaml_filename': param_file, 'use_sim_time': use_sim_time}]
    # )

    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(nav2_launch_path),
    #     launch_arguments={
    #         'map': map_file,
    #         'use_sim_time': use_sim_time,
    #         'params_file': param_file
    #     }.items()
    # )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        robot_localization_node,
        static_transform_publisher_node,
        # map_server_node,
        # nav2_launch,
    ])
