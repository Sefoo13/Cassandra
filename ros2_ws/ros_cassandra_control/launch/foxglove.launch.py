import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory('ros_cassandra_control')

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml',
            )
        )
    )

    tts_node = Node(
        package='ros_cassandra_control',
        executable='tts_service.py',
        name='text_to_speech',
        output='screen',
        parameters=[os.path.join(control_share, 'config', 'tts_service.yaml')],
    )

    asset_server = Node(
        package='ros_cassandra_control',
        executable='asset_server.py',
        name='cassandra_asset_server',
        output='screen',
        parameters=[os.path.join(control_share, 'config', 'asset_server.yaml')],
    )

    speech_recognition_node = Node(
        package='ros_cassandra_control',
        executable='speech_recognition_node.py',
        name='speech_recognition',
        output='screen',
        parameters=[
            os.path.join(control_share, 'config', 'speech_recognition.yaml')
        ],
    )

    chat_response_node = Node(
        package='ros_cassandra_control',
        executable='chat_response_node.py',
        name='chat_response',
        output='screen',
        parameters=[os.path.join(control_share, 'config', 'chat_response.yaml')],
    )

    voice_command_node = Node(
        package='ros_cassandra_control',
        executable='voice_command_node.py',
        name='voice_command',
        output='screen',
        parameters=[os.path.join(control_share, 'config', 'voice_command.yaml')],
    )

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            {'port': 8765},
            {'address': '0.0.0.0'}
        ]
    )

    return LaunchDescription([
        tts_node,
        asset_server,
        speech_recognition_node,
        chat_response_node,
        voice_command_node,
        foxglove_node,
        rosbridge_launch,
    ])
