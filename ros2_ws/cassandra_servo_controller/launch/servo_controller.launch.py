"""Launch, configure, and activate Cassandra's lifecycle servo controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    """Create the launch description with automatic lifecycle transitions."""
    config = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("cassandra_servo_controller"),
                "config",
                "servos.yaml",
            ]
        ),
        description="Path to the ROS 2 parameters YAML file",
    )
    dry_run = DeclareLaunchArgument("dry_run", default_value="false")

    controller = LifecycleNode(
        package="cassandra_servo_controller",
        executable="servo_controller",
        name="cassandra_servo_controller",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"dry_run": LaunchConfiguration("dry_run")},
        ],
    )

    configure_after_start = RegisterEventHandler(
        OnProcessStart(
            target_action=controller,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(controller),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )
    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=controller,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(controller),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            config,
            dry_run,
            controller,
            configure_after_start,
            activate_after_configure,
        ]
    )
