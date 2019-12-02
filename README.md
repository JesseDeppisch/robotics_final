# Singularities of UR3 robot arm in ROS

This project was developed to simulate the UR3 robot arm moving through singularities. \
 I also made [a YouTube video detailing this project](https://www.youtube.com/watch?v=2Sx4uq1sdFE).

## Installation

1. Copy this directory to somewhere on your Linux system. Note that you'll need to have ROS Kinetic installed. I'm using Ubuntu 16.04
2. Install all dependencies for the ROS package. These can be found in the `package.xml`, but to list them, they are `ur3_moveit_config` and `ur_gazebo`, which are part of [ROS industrial](http://wiki.ros.org/Industrial).

## Usage

Each simulation has its own Python file to run it. The way I run this project is:
1. First, launch Gazebo. Wait for Gazebo to load before proceeding to step 2
   ```bash
   roslaunch ur_gazebo ur3.launch
   ```
2. Next, launch the MoveIt planner for the UR3
   ```bash
   roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true
   ```
3. Last, launch the Python script that commands MoveIt via the Python wrapper. \
There are three different files (one for each singularity), but for example:
   ```bash
   rosrun robotics_final move_UR3_wrist.py
   ```



## Viewing my simulations
I have included the log files that I recorded with Gazebo. These allow playback of the simulations I created. These can be found under `final_ws/logs`.

To open a log is simple.
```bash
gazebo -u -p ~/final_ws/logs/[log]/gzserver/state.log
```

Here is [an additional tutorial](http://gazebosim.org/tutorials?cat=tools_utilities&tut=logging_playback), just in case.

## Ideas for the future
- Hook this up to a real UR3 arm ([tutorial](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial)) to be a more effective demonstration
- Improve the visuals in Gazebo
- Improve the reliability and repeatability of each path. This would be crucial for a great demonstration on a real arm. One way to do this would be to use the C++ API for MoveIt, instead of the Python wrapper.
