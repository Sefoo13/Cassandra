import os
import socket

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def get_mesh_base_url():
    configured_url = os.getenv("CASSANDRA_MESH_BASE_URL", "").strip()
    if configured_url:
        return configured_url.rstrip("/")

    asset_port = os.getenv("CASSANDRA_ASSET_PORT", "8080")
    detected_ip = "127.0.0.1"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            # UDP connect selects the default route without sending data.
            sock.connect(("1.1.1.1", 80))
            detected_ip = sock.getsockname()[0]
    except OSError:
        try:
            detected_ip = socket.gethostbyname(socket.gethostname())
        except OSError:
            pass

    return f"http://{detected_ip}:{asset_port}/meshes"


def generate_launch_description():
    description_path = os.path.join(
        get_package_share_directory("ros_cassandra_description")
    )
    xacro_file = os.path.join(description_path, "urdf", "robot.xacro")
    dry_run = LaunchConfiguration("dc_motor_dry_run")
    mesh_base_url = get_mesh_base_url()
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " dc_motor_dry_run:=",
                dry_run,
                " mesh_base_url:=",
                mesh_base_url,
            ]
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    package_name = "ros_cassandra_control"
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "controllers.yaml",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    forward_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    servo_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cassandra_servo_controller"),
                "launch",
                "servo_controller.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            LogInfo(msg=f"Cassandra mesh base URL: {mesh_base_url}"),
            DeclareLaunchArgument(
                "dc_motor_dry_run",
                default_value="false",
                description="Do not open I2C or write to the PCA9685",
            ),
            robot_state_publisher_node,
            controller_manager,
            diff_drive_spawner,
            joint_broad_spawner,
            forward_position_controller,
            servo_controller_launch,
        ]
    )
