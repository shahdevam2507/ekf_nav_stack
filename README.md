# EKF Navigation Stack

This repository contains a custom ROS 2 navigation stack implementing EKF-based SLAM and Localization, along with A* Global Planning and MPC control.

## Packages
- `ekfslam`: EKF SLAM and Localization nodes.
- `global_planner`: A* path planning and Model Predictive Control (MPC).
- `my_robot_description`: Robot URDF and Gazebo launch files.
- `saved_maps`: Stores generated maps.

## Build Instructions

```bash
cd ~/custom_nav_stack
colcon build
source install/setup.bash
```

## Execution Sequence

### 1. Launch Simulation
Launch the robot in the Gazebo simulation environment:
```bash
ros2 launch my_robot_description gazebo.launch.py
```
*(Note: You can also use `cylinder.launch.py` or `empty_world.launch.py` depending on the environment you wish to load.)*

### 2. PS4 Controller Teleop (Crucial Step)
To control the robot manually using a PS4 controller, run the `joy_node`:
```bash
# In a new terminal (don't forget to source install/setup.bash)
ros2 run joy joy_node
```
*(Ensure your PS4 controller is connected via Bluetooth or USB).*

### 3. Run EKF SLAM / Localization
Run the EKF nodes to perform SLAM or localization.
For SLAM:
```bash
ros2 run ekfslam ekfnode
ros2 run ekfslam ekfviz
```
For Localization (if you have an existing map):
```bash
ros2 run ekfslam ekflocnode
ros2 run ekfslam ekflocviz
```

### 4. Global Planning and Control
To navigate autonomously, run the A* planner and MPC controller:
```bash
ros2 run global_planner astar_planner
ros2 run global_planner mpc_controller
```

---
**Note:** This execution sequence is a general guide. Be sure to run these commands in separate terminals, and always remember to source the workspace (`source install/setup.bash`) in each new terminal.
