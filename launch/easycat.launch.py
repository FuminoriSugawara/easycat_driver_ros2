
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='easycat.config.xacro',
            description='URDF/XACRO description file with the axis.',
        )
    )

    description_file = LaunchConfiguration('description_file')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ethercat_easycat"),
                    "description/config",
                    description_file,
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ethercat_easycat"),
            "config",
            "controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trajectory_controller", "-c", "/controller_manager"],
    )

    # position_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["position_controller", "-c", "/controller_manager"],
    # )

    # velocity_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["velocity_controller", "-c", "/controller_manager"],
    # )

    # effort_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["effort_controller", "-c", "/controller_manager"],
    # )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        trajectory_controller_spawner,
        #position_controller_spawner,
        #velocity_controller_spawner,
        # effort_controller_spawner,
    ]

    return LaunchDescription(
        declared_arguments +
        nodes)