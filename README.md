# Swiff

This repo is a simulation of Swiff, the cleaning robot. In the environment the robot is dropped in, it will first explore its surroundings, then it will be ready to clean the area.

## Installation

1. Clone this repo into `<dev_ws>/src`. Rename `dev_ws` to your workspace name.
2. Install ros2 humble
3. Install the dependencies for this repository using `rosdep install --from-paths src --ignore-src -r -y` in `<dev_ws>`
4. Build the workspace using `colcon build --symlink-install` in `<dev_ws>`

## Running the simulation

Note: this series of commands to run the robot is not ideal, and will be fixed later, it is just a temporary situation.

1. Run the build command in the installation guide, if you have not already. Source the workspace using `source install/setup.bash`
2. Run the simulation using `ros2 launch swiff_bringup sim.launch.py`. This script spawns the robot in a gazebo environment. It also launches a teleop joystick node, so be sure to connect your joystick before running this command. If you don't have a joystick, in a new terminal, run `ros2 run teleop_twist_keyboard teleop_twist_keyboard` to control the robot using the keyboard (make sure to install this package if you don't already have it).
3. In a new terminal, run the command `ros2 launch nav2_bringup navigation_launch.py` to start the navigation stack.
4. In a new terminal, run the command `ros2 run swiff_explore_py frontier_exploration` to start the robot's exploration process.
