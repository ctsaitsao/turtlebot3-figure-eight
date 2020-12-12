# ME495 (Embedded Systems for Robotics) Homework 2

Author: Christopher Tsai

## Overview

This package contains nodes that accomplish two tasks:

1. Cause a TurtleBot to follow a configurable figure 8 trajectory. One has the option to run this code in real-life or in a Gazebo simulation. Either way it is accompanied by an RViz visualization.

2. Visualize a 2R arm that follows a trajectory where its y-position remains constant. 

## Dependencies

- turtlebot3

## Demos

TurtleBot RVIz: https://youtu.be/BLI5nBEQBdw  
TurtleBot IRL: https://youtu.be/CagvXUCvS04  
Arm RViz: https://youtu.be/47MknIazwUA  

## Usage Instructions

1. Create a new workspace and clone this repositiory.
```Shell
# Create a new workspace
mkdir -p ws/src

# clone the demonstration code
cd ws/src
git clone https://github.com/ME495-EmbeddedSystems/homework02-ctsaitsao homework2

# return to ws root
cd ../
```

2. Build the workspace and activate it.
```Shell
catkin_make install
. devel/setup.bash
```

3. To run the TurtleBot code in real life, create these three terminals:
    1. In the first one, create a ROS Master.
    ```Shell
    roscore
    ```
    2. In the second one, SSH into a TurtleBot machine and run the startup launch file.
    ```Shell
    ssh ubuntu@turtlebot.local
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```
    3. In the third, export the TurtleBot model (if this hasn't been done before) and run the main launch file.
    ```Shell
    export TURTLEBOT3_MODEL=burger
    roslaunch homework2 figure_eight.launch
    ```

4. The 2R arm code runs two main nodes, `arm_traj` (which commands the arm trajectory) and `arm_marker` (which draws markers at the end-effector of the arm). To run the 2R arm code, run:
```Shell
roslaunch homework2 arm_traj.launch
```

## Configuration Options

- TurtleBot configuration options:
    - Add the `launch_gazebo:=True` option to run the TurtleBot code in the Gazebo simulation.
    - Add the `launch_rqt_plot:=True` option to graph the x and y positions of the robot over time.
    - Add the `launch_rviz=False` option to not run RViz.

- 2R arm configuration options:
    - Add the `display_markers:=False` option to not display the markers at the end-effector frame of the arm.
    - The 2R arm can be manually controlled (without a trajectory) by running:
        ```Shell
        roslaunch homework2 view_arm.launch launch_gui:=True
        ```
      This launch file is included in `arm_traj.launch` and runs the `rviz`, `joint_state_publisher_gui`, and `robot_state_publisher` nodes.

## Calculations

Handwritten calculations for the TurtleBot control velocity inputs v and w as well as the TurtleBot's trajectory parametric equations can be found in the `doc` folder.
