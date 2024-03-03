
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from control_bolide.msg import SpeedDirection
from nav_module.utils import get_dials_ranges, crop_cmd_vel


__author__ = "Eliot CHRISTON and Raphael KHORASSANI and Loris OUMBICHE"
__status__ = "Development"
__version__ = "3.2.1"



def forward_3_dials(lidar_data:LaserScan, Kspeed:float, Kdir:float, Karg:float, mode:str="spaced", FrontRatio: float = 0.2, **args) -> SpeedDirection:
    """Return the speed and direction of the robot based on 3 dials range data."""

    # Get the ranges of the dials
    if mode == "spaced" :
        range_left, _, range_center, _, range_right = get_dials_ranges(lidar_data, n_dials=5, proportion=[70, 10, 20, 10, 70])
    else :
        range_left, range_right = get_dials_ranges(lidar_data, n_dials=2)
        _, range_center, _ = get_dials_ranges(lidar_data, n_dials=3, proportion=[1-FrontRatio, 2*FrontRatio, 1-FrontRatio])

    # Compute the mean distance of each dial
    dist_left = np.mean(range_left)
    dist_center = np.mean(range_center)
    dist_right = np.mean(range_right)

    # Argmax step
    mid_i = len(lidar_data.ranges)//2
    arg = -(np.argmax(lidar_data.ranges) - mid_i)/mid_i

    # Compute the speed command
    speed_cmd = Kspeed * dist_center

    # Compute the direction command
    dir_cmd = (- Kdir * (dist_right - dist_left) / dist_center) + (Karg * arg)

    cmd_vel = SpeedDirection(speed_cmd, dir_cmd)

    # Crop the commands
    return crop_cmd_vel(cmd_vel, speed_lim={"min":0.2, "max":1})


def forward_n_dials(lidar_data:LaserScan, Kspeed:float, Kdir:float, Karg:float, mode:str, n_dials:int=11, 
                is_spaced:bool=True, navigation_feature=np.median, FrontRatio:float = 0.2,
                use_maximize_threshold:bool = False,  maximize_threshold:float = 0.5,
                use_Kv_as_constant:bool = True,
                **args) -> SpeedDirection:
    """Return the speed and direction of the robot based on N dials range data."""

    # Get the ranges of the dials
    if is_spaced:
        range_left, _, range_center, _, range_right = get_dials_ranges(lidar_data, n_dials=5, proportion=[70, 10, 20, 10, 70])
    else :
        range_left, range_right = get_dials_ranges(lidar_data, n_dials=2)
        _, range_center, _ = get_dials_ranges(lidar_data, n_dials=3, proportion=[1-FrontRatio, 2*FrontRatio, 1-FrontRatio])

    dial_ranges = get_dials_ranges(lidar_data, n_dials=n_dials)

    dial_values = np.array([navigation_feature(dial) for dial in dial_ranges])

    # Compute the mean distance of each dial
    dist_left = navigation_feature(range_left)
    dist_center = navigation_feature(range_center)
    dist_right = navigation_feature(range_right)

    # Argmax step
    mid_i = len(dial_values)//2

    arg = -(np.argmax(dial_values) - mid_i)/mid_i

    # Compute the speed command
    speed_cmd = Kspeed * dist_center if not use_Kv_as_constant else Kspeed

    # Compute the direction command

    if mode == "Classic" :
        dir_cmd = - Kdir * (dist_right - dist_left) / dist_center + Karg * arg

    elif mode == "Ponderated" :
        dir_cmd = (- Kdir * (dist_right - dist_left) / dist_center + Karg * arg) / (Kdir + Karg)

    elif mode == "LeftRightDivision" : 
        dir_cmd = - dist_right/dist_left - 1 if dist_right > dist_left else dist_left/dist_right - 1
        dir_cmd = (Kdir * dir_cmd / dist_center + Karg * arg) / (Kdir + Karg)

    elif mode == "PonderatedWithoutDistanceDivision":
        dir_cmd = (- Kdir * (dist_right - dist_left)  + Karg * arg) / (Kdir + Karg)

    elif mode == "PonderatedRelative":
        dir_cmd = (- Kdir * (dist_right - dist_left)  + Karg * arg) / (Kdir + Karg)**0.3

    else :
        rospy.logwarn("The navigation mode is unknown. Here mode = {}".format(mode))
        dir_cmd = 0

    if use_maximize_threshold :
        if abs(dir_cmd) > maximize_threshold :
            dir_cmd = -1 if dir_cmd < 0 else 1

    cmd_vel = SpeedDirection(speed_cmd, dir_cmd)

    # Crop the commands
    return crop_cmd_vel(cmd_vel, speed_lim={"min":0.005, "max":1})
