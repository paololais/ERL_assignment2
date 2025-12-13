# Experimental Robotics Laboratory – Assignment 2

## Overview
This package has been developed for the *Experimental Robotics Laboratory* course  
(Master’s Degree in Robotics Engineering, University of Genoa).

## Installation
- Clone the package inside your ROS2 workspace src folder:
```bash
cd ~/ros2_ws/src/
git clone https://github.com/paololais/ERL_assignment2.git
```

- Build the package and source:
```bash
cd ~/ros2_ws
colcon build
source install/local_setup.bash
```

## Usage
- Launch the Gazebo simulation and Rviz:
```bash
ros2 launch assignment2 spawn_robot_aruco.launch.py