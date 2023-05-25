from launch.actions import GroupAction, DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration as LC
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

import os

robot = 'tempest'

config = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'config',
        LC("robot_yaml")
    ])

def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions=[
                # declare the launch args here
                DeclareLaunchArgument(
                    "robot", 
                    default_value=robot,
                    description="Name of the vehicle",
                ),

                DeclareLaunchArgument(
                    "control_config_file",
                    default_value=["control_config_", LC("robot"), ".rviz"]
                ),

                DeclareLaunchArgument('robot_yaml', default_value=[LC("robot"), '.yaml']),

                # start rviz
                Node(
                    package='rviz2',
                    executable='rviz2',
                    # DONT USE IT RENAMES ALL OF THE CHILD NODES CREATED FOR THE PLUGINS
                    # name='riptide_rviz', 
                    on_exit=Shutdown(),
                    arguments=[
                        "-d", 
                        PathJoinSubstitution([
                            os.path.expanduser("~"), "osu-uwrt", "riptide_release", 
                            "src", "riptide_gui", "riptide_rviz", LC("control_config_file")
                        ])
                    ]
                ),

                # send the rest into the tempest namespace
                PushRosNamespace(["/", LC('robot')]),
                
                Node(
                    package="riptide_rviz",
                    executable="MarkerPublisher.py",
                    name="marker_publisher",
                    output="screen"
                )
            ]
        )
    ])

