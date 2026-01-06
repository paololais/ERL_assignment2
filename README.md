# Experimental Robotics Laboratory – Assignment 2

## Overview

This ROS2 package has been developed for the *Experimental Robotics Laboratory* course  
(Master’s Degree in Robotics Engineering, University of Genoa).

The package implements an autonomous robotic system that uses **PlanSys2** to plan and execute a sequence of actions for detecting and processing **ArUco markers** in the environment.

The robot is equipped with a camera and is capable of navigating in a known map, exploring predefined waypoints, detecting markers, and visiting them in a specific order based on their IDs.

---

## Main Objectives

The package allows the robot to:

- Explore the environment to detect all ArUco markers
- Store detected marker IDs and positions
- Plan actions using **PlanSys2**
- Visit markers in **ascending ID order**
- Center the marker in the camera image
- Annotate and publish the processed image (simulated picture-taking)

The behavior is an extension of *Assignment 1*, enhanced with symbolic planning and action execution.

---

## Planning Approach

The task planning is implemented using **PlanSys2**, with:

- A **PDDL domain** describing actions such as moving, detecting markers, and taking pictures
- A **PDDL problem** defining the initial state and goals
- Dedicated ROS2 action nodes implementing each planning action

The robot can visit a set of predefined waypoints to facilitate marker detection:

- (-6.0, -6.0)
- (-6.0,  6.0)
- ( 6.0, -6.0)
- ( 6.0,  6.0)

---


## Requirements

- **ROS2 Jazzy**
- **Ubuntu 24.04**
- **PlanSys2** for task planning and execution
- **Gazebo Harmonic** for simulation
- **ArUco marker detection**

Although the package was developed and tested on ROS2 Jazzy, it can be adapted to other ROS2 distributions with minor changes.

---


## Simulation

The assignment is implemented and tested in **simulation**, using the provided Gazebo world file:

- `simple_world.sdf`

The package includes a robot model `mogi_bot.urdf` – a 2-wheel differential-drive robot that relies on a standard **diff-drive plugin**.

The approach was also tested on a **physical ROSBot**, successfully achieving the expected behavior.  
The real-robot implementation can be found in the `rosbot` branch, where some modifications were introduced to improve robustness in real-world conditions (e.g. perception noise and navigation accuracy).

---

### Launch File

A custom main launch file named **assignment2.launch.py** is provided to:
- Load the simple_world.sdf world.
- Spawn the mogi_bot;
- Load the rviz file configured so that navigation and camera info are provided to the user;
- Initialize all required ROS2 interfaces for the simulation.
- Launch all the required nodes and launch-modules to allow localication, navigation, planning and robot functionalities;

# Nodes Description

### `getplan.cpp`

This node:

- Interacts with the PlanSys2 planner
- Requests a plan based on the current PDDL problem
- Outputs the generated plan

---

### `getplan_and_execute.cpp`

This node:

- Requests a plan from PlanSys2
- Automatically executes the generated plan
- Monitors execution feedback and completion
- It gets the markers and their associated IDs and reorders them
- Updates the PDDL problem with the new predicates and goal and executes the new plan

---

## PlanSys2 Action Nodes (C++)

Located in `src/actions/`, each node implements a specific symbolic action:

### `move_action_node.cpp`
- Moves the robot to a specified waypoint or target position
- Interfaces with the navigation stack

### `detect_marker_action_node.cpp`
- Triggers the marker detection phase
- Updates the planning state once detection is completed

### `take_picture_action_node.cpp`
- Simulates taking a picture of the marker
- Requests image processing

### `process_last_image_action_node.cpp`
- Annotates the last captured image
- Publishes the processed image

### `finalize_detection_phase_action_node.cpp`
- Signals the end of the detection phase
- Updates the knowledge base in PlanSys2

---

## Custom Messages

### `MarkerDetection.msg`

Used to communicate marker detection results, including:

- Marker ID
- Detection status
- Relative position or pose

---
# Configuration and Additional Files

This document describes the main configuration files and resources used by the package.

---

## Configuration Files (`config/`)

### `amcl_localization.yaml`
- Parameters for AMCL localization
- Particle filter configuration
- Sensor and motion model tuning

### `navigation.yaml`
- Configuration for the Navigation2 stack
- Planner, controller, and recovery behaviors

### `waypoints.yaml`
- Defines the predefined waypoints used for marker detection
- Coordinates used by the planner and navigation actions

---

## Launch Files (`launch/`)

### `assignment.launch.py`
- Main launch file
- Starts the full system (simulation, planning, perception, navigation)

### `distributed_actions.launch.py`
- Launches PlanSys2 action nodes
- Enables distributed execution of actions

### `localization.launch.py`
- Starts AMCL localization
- Loads the map and localization parameters

### `navigation.launch.py`
- Launches the Navigation2 stack

### `spawn_robot_aruco.launch.py`
- Spawns the robot in Gazebo
- Allows selection of the robot model via `model_arg`

---

## Maps (`maps/`)

- `map_of_world.yaml`  
- `map_of_world.pgm`  

Used for localization and navigation. The map was generated by manually navigating the robot in the environment and finally save the map by executing:
``` bash
ros2 run nav2_map_server map_saver_cli -f map_of_world 
```

---

## PDDL Files (`pddl/`)

### `domain.pddl`
- Defines actions, predicates, and types used by PlanSys2

### `problem.pddl`
- Defines the initial state and goals of the planning problem

---
# Demo videos
### Simulation
[![Watch the demo](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://youtu.be/MoiHaN4X87o)

### ROSbot
Still not provided

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
- Launch the Gazebo simulation, Rviz, calculate the plan and execute it:
```bash
ros2 launch assignment2 assignment.launch.py
```
