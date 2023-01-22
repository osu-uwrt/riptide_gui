from launch.actions import GroupAction, DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

import os

robot = 'tempest'

config = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'config',
        LaunchConfiguration("robot_yaml")
    ])

# path to rviz config
rviz_cfg_path = os.path.join(
    os.path.expanduser("~"),
    "osu-uwrt", "development", "software", "src",
    "riptide_gui", "riptide_rviz" , "control_config.rviz")

def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions=[
                # declare the launch args here
                DeclareLaunchArgument(
                    "robot", 
                    default_value="tempest",
                    description="Name of the vehicle",
                ),

                DeclareLaunchArgument('robot_yaml', default_value=[LaunchConfiguration("robot"), '.yaml']),

                # start rviz
                Node(
                    package='rviz2',
                    executable='rviz2',
                    # DONT USE IT RENAMES ALL OF THE CHILD NODES CREATED FOR THE PLUGINS
                    # name='riptide_rviz', 
                    on_exit=Shutdown(),
                    arguments=["-d", rviz_cfg_path]
                ),

                # send the rest into the tempest namespace
                PushRosNamespace(["/", LaunchConfiguration('robot')]),

                # start the thruster wrench visualizer
                Node(
                    package="riptide_controllers2",
                    executable="thruster_wrench_publisher",
                    name="thruster_wrench_publisher",
                    output="screen",
                    parameters=[
                        {"vehicle_config": config},
                        {"robot": robot},
                    ]
                ),

                Node(
                    package="riptide_controllers2",
                    executable="thruster_test",
                    name="thruster_test",
                    output="screen",
                    parameters=[
                        {"vehicle_config": config},
                        {"robot": robot},
                    ]
                ),

                Node(
                    package="riptide_controllers2",
                    executable="calibrate_buoyancy",
                    name="calibrate_buoyancy",
                    output="screen",
                    parameters=[
                        {"vehicle_config": config},
                        {"robot": robot},
                    ]
                ),

                Node(
                    package="riptide_controllers2",
                    executable="calibrate_drag",
                    name="calibrate_drag",
                    output="screen",
                    parameters=[
                        {"vehicle_config": config},
                        {"robot": robot},
                    ]
                ),
            ]
        )
    ])

