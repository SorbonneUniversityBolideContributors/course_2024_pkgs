# Control ROS Package

This repository contains the perception ROS package for the COVAPSY Autonomous RC Car Race project, developed by the Sorbonne University team.

## Project Overview

- **Main Git Repository for the 2024 Race:** [Course_2024](https://github.com/Pfecourse/Course_2024)

### Project Packages

This control package is one of the three main packages in the project:

1. **Perception (current):** [perception_bolide](https://github.com/SorbonneUniversityBolideContributors/course_2024_pkgs/tree/main/perception_bolide)
2. **Planning:** [planning_bolide](https://github.com/SorbonneUniversityBolideContributors/course_2024_pkgs/tree/main/planning_bolide)
3. **Control:** [control_bolide](https://github.com/SorbonneUniversityBolideContributors/course_2024_pkgs/tree/main/control_bolide)

## Package Content

In this package, you will find the first nodes in the software architecture of the COVAPSY project. These nodes are responsible for sensor data publication and processing.

### Nodes

#### `src_publisher/` contains the source code for the publisher nodes (All this nodes need to be launched on the robot):

- `cam_publisher.py`: Retrieves the camera data and publish it.
    - topic: `raw_image_data`
    - msg type: `sensor_msgs.msg.Image`

- `lidar_publisher.py`: Retrieves the lidar data, crop it and publish it.
    - topic: `raw_lidar_data`
    - msg type: `sensor_msgs.msg.LaserScan`

- `STM32_sensors_pub.py`: Retrieves the IMU, IR, US and optical fork raw data from the STM32 and publish them. The data is then preprocessed by the `IMU_publisher.py`, `rear_sensor_publisher.py` and `fork_publisher.py` nodes.
    - topic: `STM32_sensors_topic`
    - msg type: `std_msgs.msg.Float32MultiArray`

- `IMU_publisher.py`: Retrieves the IMU data from the `STM32_sensors_topic` convert it to SI units and publish it.
    - topic: `raw_imu_data`
    - msg type: `sensor_msgs.msg.Imu`

- `rear_sensor_publisher.py`: Retrieves the IR and US data from the `STM32_sensors_topic` convert them to SI units, crop them and publish them.
    - topic: `raw_rear_range_data`
    - msg type: `perception_bolide.msg.MultipleRange`

- `fork_publisher.py`: Retrieves the optical fork data from the `STM32_sensors_topic` convert it to SI units and publish it.
    - topic: `raw_fork_data`
    - msg type: `perception_bolide.msg.ForkSpeed`

#### `src_vizu/` contains the source code for the visualization nodes (All this nodes can only be launched on your computer):

- `lidar_vizu.py`: Used to visualize the lidar data. Plot the data published on the `raw_lidar_data` and `lidar_data` topics.

- `gui/`: this folder contains all the scripts needed to run a GUI used to change the ROS parameters of the nodes. For more information please refer to the [GUI_README.md](src_vizu/gui/GUI_README.md) file.

#### `src_process/`: contains the source code for the processing nodes:

- `lidar_process.py`: Used to process the lidar data. It use temporal and spatial filters to remove noise. The parameters of the filters can be changed using the GUI or rosparam.
    - topic subscribed: `raw_lidar_data`
    - topic published: `lidar_data`
    - msg type: `sensor_msgs.msg.LaserScan`

- `camera_info.py`: TODO

- `calibrate_color.py`: TODO

- `fork_and_imu_odom_process.py`: Used in the gmapping launch to send the odometry information and frame.
    - topic subscribed: `raw_fork_data`, `raw_imu_data`
    - topic published: `Odom`
    - msg type: `nav_msgs.msg.Odometry`

#### `msg/` contains the custom message definition:

- `MultipleRange.msg`: The message used to send the IR and US data from the `rear_sensor_publisher.py` node.
    ```
    sensor_msgs/Range IR_rear_left
    sensor_msgs/Range IR_rear_right
    sensor_msgs/Range Sonar_rear
    ```

- `ForkSpeed.msg`: The message used to send the optical fork data from the `fork_publisher.py` node.
    ```
    std_msgs/Header header
    std_msgs/Float32 speed
    ```

- `CameraInfo.msg`: The message used to send the camera data from the `cam_publisher.py` node.
    ```
    bool wrong_way
    string front_color
    string left_color
    string right_color
    ```
#### `rviz/` contains the rviz configuration files:

- `rviz_config_reel.rviz`: Used to set rviz configuration for the real robot

- `rviz_config_simu.rviz`: Usef to set rviz configuration for the robot in simulation


#### `launch/` contains the launch files:

- `perception.launch`: Used to launch all the publisher nodes and process nodes.
- `main_publisher.launch`: Used to launch all the publisher nodes without the processing nodes.
- `process.launch`: Used to launch all the processing nodes without the publisher nodes (Used for simulation).
- `gmapping.launch`: Used to launch the gmapping_slam package that works better than hector slam in our case as we have odometry.
- `hector_lidar_slam.launch`: Used to launch the hector_slam package using only the lidar data (usefull when there is no other way than lidar to get the odometry).


#### `urdf_files/` contains the urdf files:

- `bolide.urdf`: The urdf file of the bolide robot (not complete yet).
