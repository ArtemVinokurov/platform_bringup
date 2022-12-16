import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():


    urdf_file = os.path.join(get_package_share_directory('platform_description'), 'urdf', 'vertical_robot.urdf')

    serial_port_param_name = "device"
    use_sim_param_name = "use_sim"

    device = LaunchConfiguration(serial_port_param_name)
    use_sim = LaunchConfiguration(use_sim_param_name)

    serial_port_arg = DeclareLaunchArgument(
        serial_port_param_name,
        default_value="/dev/ttyACM0",
        description='Port to communicate with OpenCR')

    use_sim_arg = DeclareLaunchArgument(
        use_sim_param_name,
        default_value="false",
        description='Use only rviz')



    

    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()
    
    robot_description = {'robot_description': robot_description_config}
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        remappings=[("/robot_description", "/platform/robot_description"), ("/joint_states", "/platform/joint_states")],
        parameters=[robot_description],
        condition=UnlessCondition(use_sim)
    )



    serial_connection_node = Node(
        package='platform_bringup',
        executable='serial_connection',
        output='screen',
        parameters=[{serial_port_param_name : device}],
        condition=UnlessCondition(use_sim)
    )

    publish_joint_state_node = Node(
        package='platform_bringup',
        executable='publish_joint_state.py',
        output='screen',
        condition=UnlessCondition(use_sim)
        
    )


    platform_robot_odom = Node(
        package='platform_pkg',
        executable='walk_robot_odom.py',
        output='screen',
        condition=UnlessCondition(use_sim)
    )


    rviz_config = os.path.join(get_package_share_directory('platform_description'), 'config', 'rviz.rviz')


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[robot_description],
        condition=UnlessCondition(use_sim)
    )


    platform_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('platform_pkg'), 'launch', 'platform_model.launch.py'])),
        condition=IfCondition(use_sim)
    )

    return LaunchDescription(
        [serial_port_arg,
         use_sim_arg,
         #robot_state_publisher,
         #publish_joint_state_node,
         #platform_robot_odom,
         serial_connection_node,
         platform_model_launch,
         rviz_node,
         #rviz_arg,
        ]
    )