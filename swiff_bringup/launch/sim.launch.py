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
                "simple.world",
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
                get_package_share_directory("swiff_bringup"), "config", "map.rviz"
            ),
        ],
        output="screen",
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                )
            ]
        )
    )

    delayed_launch = TimerAction(period=5.0, actions=[rviz2, slam_toolbox])

    return LaunchDescription(
        [joystick, robot_state_publisher, gazebo, spawn_turtlebot3, delayed_launch]
    )
