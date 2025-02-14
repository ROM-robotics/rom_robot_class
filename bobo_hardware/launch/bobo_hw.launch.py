from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get URDF via xacro
    bobo_model = os.getenv('BOBO_MODEL', 'robot2') # robot1 , robot2 you should use robot1 cause of light weight.
    robot_urdf_name = bobo_model+'_complete.urdf.xacro'
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("bobo_description"), "urdf", robot_urdf_name]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("bobo_hardware"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("bobo_hardware"), "rviz", "diffbot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers_yaml],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diffbot_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output='both',
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_diffbot_base_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffbot_base_controller_spawner],
        )
    )

    delay_gpio_after_diffbot_base_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diffbot_base_controller_spawner,
            on_exit=[gpio_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        #delay_rviz_after_joint_state_broadcaster_spawner,
        delay_diffbot_base_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_gpio_after_diffbot_base_controller_spawner,
    ]

    return LaunchDescription(nodes)
