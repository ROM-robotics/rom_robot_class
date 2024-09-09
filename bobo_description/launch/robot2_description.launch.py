import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    bobo_model = os.getenv('BOBO_MODEL', 'robot2') # robot1 , robot2, sim_robot2
    robot_urdf_name = bobo_model+'_complete.urdf.xacro'

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('bobo_description'))
    xacro_file = os.path.join(pkg_path,'urdf', robot_urdf_name)
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_node = Node(
        name="joint_state_publisher",
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])