# Waypoint Manager

A ROS package for managing waypoints and navigation for mobile robots. This package allows users to create waypoints manually or automatically as the robot moves through an environment, and then navigate between these waypoints efficiently.

## Features

- Manual waypoint creation at the robot's current position
- Automatic waypoint creation based on distance and angle thresholds
- Interactive visualization of waypoints in RViz
- Waypoint persistence to disk using YAML format
- Efficient path planning between waypoints using A* algorithm
- Integration with move_base for navigation
- Interactive markers for easy waypoint selection and navigation

## Requirements

- docker
- ROS Noetic
- yaml-cpp
- tf2
- move_base
- interactive_markers

## Installation

1. Clone this package into your docker workspace along with anscer robotics :
```bash
cd ~/catkin_ws/src
git clone https://github.com/anscer/AR100.git
git clone https://github.com/phdraushan/waypoint_manager.git
# building
```

2. Install dependencies:
```bash
cd ..
# checking dependencies
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
cd ~/catkin_ws
catkin_make
```
## Launch file sequence

1. Start anscer bringup
```bash
roslaunch start_anscer start_anscer.launch
```
2. Start anscer navigaiton
```bash
roslaunch anscer_navigation anscer_navigation.launch
```
3. Start the waypoint manager:
```bash
roslaunch waypoint_manager waypoint_manager.launch
```

## Services

1. Create waypoints:
   - Manual: Use the `create_waypoint` service
   - Automatic: Move the robot around, waypoints will be created automatically when distance or angle thresholds are met
  

2. Navigate to waypoints:
   - Click on waypoint markers in RViz
   - Use the `navigate_to_waypoint` service

3. Get Waypoints:
   - Array of all waypoints


## Parameters

- `position_threshold` (default: 0.5m): Minimum distance between waypoints
- `distance_threshold` (default: 2.0m): Distance threshold for automatic waypoint creation
- `angle_threshold` (default: 45 degrees): Angle threshold for automatic waypoint creation

## Visualization

Waypoints are visualized in RViz using interactive markers:
- Manual waypionts : Red arrows
- Automatic waypoints : Green arrows
- Click on a waypoint marker to navigate to it

## Waypoint Storage

- `waypoints.yaml`: Persistent storage for waypoints
- Manual and automatic wayponts named differently