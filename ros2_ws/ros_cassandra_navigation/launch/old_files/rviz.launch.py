#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    xacro_file = "robot.urdf"
    package_description = "ros_cassandra"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", xacro_file)

    rviz_config = os.path.join(get_package_share_directory(package_description), "rviz", "robot.rviz")
    # Robot State Publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': False, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )
    
    rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
      name='rviz_node',
      arguments=["-d", rviz_config],
      parameters=[{'use_sim_time': False}],
    )

    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node,
            rviz_node
        ]
    )
