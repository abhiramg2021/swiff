import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory("swiff_bringup"))
    xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
    robot_description_config = Command(["xacro ", xacro_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_config,
                "use_sim_time": True,
            }
        ],
    )

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

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
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
        actions=[rviz2, gazebo, spawn_robot, joystick, robot_state_publisher],
    )

    return LaunchDescription([navigation, delayed_launch])
