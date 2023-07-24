from launch.actions import GroupAction, DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration as LC
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

import os

robot = 'puddles'

config = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'config',
        LC("robot_yaml")
    ])

rviz_pkg_dir = get_package_share_directory('riptide_rviz')

riptide_rviz_src = os.path.join(rviz_pkg_dir[: rviz_pkg_dir.find("install") - 1], "src", "riptide_gui", "riptide_rviz")

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
                            riptide_rviz_src,
                            LC("control_config_file")
                        ])
                    ],
                    # prefix=["gdbserver localhost:3000"]
                ),

                # send the rest into the tempest namespace
                PushRosNamespace(["/", LC('robot')]),

                # start the thruster wrench visualizer
                Node(
                    package="riptide_controllers2",
                    executable="thruster_wrench_publisher",
                    name="thruster_wrench_publisher",
                    output="screen",
                    parameters=[
                        {"vehicle_config": config},
                        {"robot": LC("robot")},
                    ]
                ),

                Node(
                    package="riptide_controllers2",
                    executable="thruster_test",
                    name="thruster_test",
                    output="screen",
                    parameters=[
                        {"vehicle_config": config},
                        {"robot": LC("robot")},
                    ]
                ),

                Node(
                    package="riptide_controllers2",
                    executable="calibrate_buoyancy",
                    name="calibrate_buoyancy",
                    output="screen",
                    parameters=[
                        {"vehicle_config": config},
                        {"robot": LC("robot")},
                    ]
                ),

                Node(
                    package="riptide_controllers2",
                    executable="calibrate_drag",
                    name="calibrate_drag",
                    output="screen",
                    parameters=[
                        {"vehicle_config": config},
                        {"robot": LC("robot")},
                    ]
                ),
                
                Node(
                    package="riptide_rviz",
                    executable="MarkerPublisher.py",
                    name="marker_publisher",
                    output="screen",
                    parameters = [
                        os.path.join(get_package_share_directory("riptide_rviz"), "config", "markers.yaml")
                    ]
                )
            ]
        )
    ])

