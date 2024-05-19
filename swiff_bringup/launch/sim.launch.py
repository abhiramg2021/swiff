import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    os.environ["TURTLEBOT3_MODEL"] = "waffle"

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("swiff_bringup"),
                "launch",
                "joystick.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("turtlebot3_gazebo"),
                "launch",
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world": os.path.join(
                get_package_share_directory("swiff_bringup"),
                "worlds",
                "small.world",
            )
        }.items(),
    )

    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("turtlebot3_gazebo"),
                    "launch",
                    "spawn_turtlebot3.launch.py",
                )
            ]
        )
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("swiff_bringup"),
                "config",
                "nav2_default_view.rviz",
            ),
        ],
        output="screen",
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "True",
            "slam": "True",
            "map": "",
            "use_composition": "False",
            "params_file": os.path.join(
                get_package_share_directory("swiff_bringup"),
                "config",
                "nav2_params.yaml",
            ),
        }.items(),
    )

    # this is needed for some reason for the nodes the launch properly, and close properly when ctrl + c is used.
    # otherwise there seems to be some conflict between launching gazebo and navigation node at the same time.

    delayed_launch = TimerAction(
        period=1.0,
        actions=[rviz2, gazebo, spawn_turtlebot3, joystick, robot_state_publisher],
    )

    return LaunchDescription([navigation, delayed_launch])
