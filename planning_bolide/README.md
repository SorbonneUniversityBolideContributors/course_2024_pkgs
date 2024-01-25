# Planning_bolide ROS Package

This repository contains the planning ROS package for the COVAPSY Autonomous RC Car Race project, developed by the Sorbonne University team.

## Project Overview

- **Main Git Repository for the 2024 Race:** [Course_2024](https://github.com/SorbonneUniversityBolideContributors/Course_2024)

### Project Packages

This control package is one of the three main packages in the project:

1. **Perception:** [perception_bolide](https://github.com/SorbonneUniversityBolideContributors/course_2024_pkgs/tree/main/perception_bolide)
2. **Planning (current):** [planning_bolide](https://github.com/SorbonneUniversityBolideContributors/course_2024_pkgs/tree/main/planning_bolide)
3. **Control:** [control_bolide](https://github.com/SorbonneUniversityBolideContributors/course_2024_pkgs/tree/main/control_bolide)

## Package Content

This packages is in between the perception and the control. It is responsible for the command generation by using odometry, SLAM, path planning, obstacle avoidance and also reactive algorithms.

### Nodes

#### `src_nav/` contains the source code for the navigation nodes and the teleoperation nodes:

- `nav_lidar_argmax.py`: TODO
- `nav_lidar.py`: TODO
- `nav_3states.py`: TODO. (For architecture details please refer to the [SM_architecture.pdf](documentation/SM_architecture.pdf) file). 

#### `src_teleop/` contains the source code for the teleoperation nodes:

- `keyboard_emergency_brake.py`: Node to launch on your computer (due to the pynput dependency). It allows to stop the car by pressing the `space` key. It overrides all navigation/teleoperation nodes.
- `teleop_keyboard.py`: Node to launch on your computer (due to the pynput dependency). It allows to teleoperate the car by pressing the `z`, `q`, `s` and `d` keys (Arrows keys also work).
- `teleop_PS4controller.py`: Node to launch on the robot (because the controller is connected to the robot). It allows to teleoperate the car with a PS4 controller. The controller must be connected to the robot via bluetooth (please refer to the [robot_setup.md](https://github.com/SorbonneUniversityBolideContributors/Course_2024/blob/main/Robot_setup.md) file). To teleoperate the car, press the `R2` trigger (accelerate), the `L2` trigger (first brake, then reverse) and use the left joystick to steer the car. `Options` is used to stop the teleoperation.

### Launch Files

#### Please find the launch files in the `launch/` folder:

- `ready_for_nav.launch`: Used to launch the controller node and all the needed sensors (lidar, camera, IMU, IR, US, optical fork). This allows to launch the navigation nodes. (More information about the perception nodes launched can be found in the `README.md` file of the [perception_bolide](https://github.com/SorbonneUniversityBolideContributors/course_2024_pkgs/tree/main/perception_bolide) package).