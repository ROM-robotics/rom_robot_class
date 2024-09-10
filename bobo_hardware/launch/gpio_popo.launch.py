from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    # Get URDF via xacro
    use_rviz = LaunchConfiguration('use_rviz')
    use_rqt = LaunchConfiguration('use_rqt')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                # [FindPackageShare("bobo_hardware"), "urdf", "diffbot.urdf.xacro"]
                [FindPackageShare("bobo_description"), "urdf", "robot2_complete.urdf.xacro"]
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
        [FindPackageShare("bobo_description"), "rviz", "diffbot.rviz"]
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
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    rqt_pub = Node(
        package="rqt_publisher",
        executable="rqt_publisher",
        condition=IfCondition(LaunchConfiguration('use_rqt')),
        name="rqt",
        output="log",
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
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true', description='Use rviz.'),
            DeclareLaunchArgument('use_rqt', default_value='true', description='Use RQT.'),
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_diffbot_base_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_gpio_after_diffbot_base_controller_spawner,
            rqt_pub,
        ]
        )
       
      

 
