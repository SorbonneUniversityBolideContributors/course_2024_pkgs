# Control ROS Package

This repository contains the perception ROS package for the COVAPSY Autonomous RC Car Race project, developed by the Sorbonne University team.

## Project Overview

- **Main Git Repository for the 2024 Race:** [Course_2024](https://github.com/Pfecourse/Course_2024)

### Project Packages

This control package is one of the three main packages in the project:

1. **Perception (current):** [perception_bolide](https://github.com/Pfecourse/perception_bolide)
2. **Planning:** [planning_bolide](https://github.com/Pfecourse/planning_bolide)
3. **Control:** [control_bolide](https://github.com/Pfecourse/control_bolide)

## Package Contents

In this package, you will find the first nodes in the software architecture of the COVAPSY project. The perception nodes are located in the `src` folder and are responsible for sensor data processing and object detection.

## Getting Started

To integrate this control ROS package into your development environment, clone the repository using the following command:

```bash
git clone https://github.com/Pfecourse/perception_bolide.git
```

## Usage

To launch the perception nodes, run the following command:

```bash
roslaunch perception_bolide perception.launch
```