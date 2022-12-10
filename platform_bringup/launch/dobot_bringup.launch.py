import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os


def generate_launch_description():


    urdf_file = os.path.join(get_package_share_directory('platform_description'), 'urdf', 'vertical_robot.urdf')

    serial_port_param_name = 'serial_port'

    #serial_port = LaunchConfiguration(serial_port_param_name)

    serial_port_arg = DeclareLaunchArgument(
        serial_port_param_name,
        default_value='/dev/ttyACM0',
        description='Port to communicate with OpenCR')

    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()
    
    robot_description = {'robot_description': robot_description_config}
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        remappings=[("/robot_description", "/platform/robot_description"), ("/joint_states", "/platform/joint_states")],
        parameters=[robot_description]
    )



    serial_connection_node = Node(
        package='platform_bringup',
        executable='serial_connection.py',
        output='screen'
    )

    publish_joint_state_node = Node(
        package='platform_bringup',
        executable='publish_joint_state.py',
        output='screen'
    )


    platform_robot_odom = Node(
        package='platform_pkg',
        executable='walk_robot_odom.py',
        output='screen'
    )





    rviz_config = os.path.join(get_package_share_directory('platform_pkg'), 'config', 'rviz.rviz')


    rviz__node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[robot_description]
    )

    return LaunchDescription(
        [robot_state_publisher,
         platform_robot_odom,
         #rviz__node,
         #rviz_arg,
        ]
    )