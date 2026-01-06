# Experimental Robotics Laboratory – Assignment 1

## Overview

This package has been developed for the *Experimental Robotics Laboratory* course  (Master’s Degree in Robotics Engineering, University of Genoa).

The goal of the project is to develop a **ROS2 package** that allows a mobile robot equipped with a camera to autonomously **detect, visit and process ArUco markers** in the environment, following a planned sequence of actions.

The robot implements the same general approach of *Assignment 1*, extended with task planning through **PlanSys2**.

---

# Package Description

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


## Technologies and Requirements

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



## Notes

This project was developed as a **group assignment** (4–5 people).  
Different aspects of the system were addressed in parallel, including simulation setup, navigation, marker detection and task planning.


---

### World
The file `simple_world.sdf` defines an environment containing **four ArUco markers** and some walls, so that the robot has to navigate and localize itself using the provided map and sensors.

### Marker Generation
Markers are generated using the package:

https://github.com/SaxionMechatronics/ros2-gazebo-aruco

Run:

```bash
ros2 run ros2_aruco aruco_generate_marker
```

Before generating the markers, minor modifications were applied to ensure the use of DICT_ARUCO_ORIGINAL.

The generated marker (a cube with 6 faces) replaces the texture of the aruco_box model from the same repository (gz-world/aruco_box).

### Robot

The simulated robot used is the mogi_bot (as introduced in the course). The robot is equipped with the following sensors:

1. A standard camera sensor is used for image acquisition;
2. A lidar used for navigation and localization tasks; 

### Launch File

A custom main launch file named **assignment2.launch.py** is provided to:
- Load the simple_world.sdf world.
- Spawn the mogi_bot;
- Load the rviz file configured so that navigation and camera info are provided to the user;
- Initialize all required ROS2 interfaces for the simulation.
- Launch all the required nodes and launch-modules to allow localication, navigation, planning and robot functionalities;

Launch file location:
ERLassignment2/launch/assignment2.launch.py

# Nodes Description

This package includes both **Python** and **C++** ROS2 nodes. Python nodes mainly handle perception and high-level coordination, while C++ nodes implement **PlanSys2 actions**.

---

## Python Nodes

### `detect_aruco_node.py`

This node is responsible for:

- Subscribing to the camera image topic
- Detecting ArUco markers in the image
- Publishing marker detections using a custom message (`MarkerDetection.msg`)
- Providing marker ID and relative pose information

---

### `planning.py`

This node acts as a high-level coordinator:

- Interfaces with PlanSys2
- Requests plan generation
- Triggers plan execution
- Monitors the planning and execution process

---

## C++ Nodes

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

Used for localization and navigation.

---

## PDDL Files (`pddl/`)

### `domain.pddl`
- Defines actions, predicates, and types used by PlanSys2

### `problem.pddl`
- Defines the initial state and goals of the planning problem

---

## Robot Description

### URDF (`urdf/`)
- Robot model definition
- Sensor and plugin configuration

### Meshes (`meshes/`)
- 3D models for visualization and simulation

---

## Visualization (`rviz/`)

- Preconfigured RViz setups for:
  - Robot visualization
  - Navigation
  - Marker detection

---

## Worlds (`worlds/`)

### `simple_world.sdf`
- Gazebo world used for simulation and testing

---
# Demo videos
### Simulation
https://github.com/paololais/ERLassignment2/blob/main/video/demo.mp4

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
- Launch the Gazebo simulation and Rviz:
```bash
ros2 launch assignment2 assignment.launch.py
```

- Run the node:
```bash
ros2 run assignment2 detect_aruco_node
```