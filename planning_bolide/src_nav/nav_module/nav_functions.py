
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from control_bolide.msg import SpeedDirection
from perception_bolide.msg import CameraInfo


def get_dials_ranges(lidar_data:LaserScan, n_dials:int=3, proportion:list=None) -> np.ndarray:
    if not(proportion is None) and len(proportion) != n_dials:
        rospy.logwarn("The proportion list must have the same length as the number of dials., Here {} != {}".format(len(proportion), n_dials))
        proportion = None
    if len(lidar_data.ranges) < n_dials:
        rospy.logwarn("The number of dials is greater than the number of points in the lidar data. Here {} > {}".format(n_dials, len(lidar_data.ranges)))
        return None

    if proportion is None:
        proportion = [1] * n_dials

    # get the indices of the dials using the proportion
    # first scale the proportion to sum to the number of points
    # normalize the proportion
    proportion = len(lidar_data.ranges) * (np.array(proportion) / sum(proportion))
    # then round the proportion to get the number of points per dial
    proportion = np.round(proportion).astype(int)

    # get the indices of the dials
    dials_indices = np.cumsum(proportion)[:-1]

    # add 0 and the last index to the indices
    dials_indices = np.insert(dials_indices, 0, 0)
    dials_indices = np.append(dials_indices, len(lidar_data.ranges))

    # get the ranges of the dials
    dials = []
    for i in range(n_dials):
        dials.append(lidar_data.ranges[dials_indices[i]:dials_indices[i+1]])

    return np.array(dials)


def crop_cmd_vel(
        cmd_vel:SpeedDirection, 
        speed_lim:dict={"min":-1, "max":1}, 
        direction_lim:dict={"min":-1, "max":1}
        ) -> SpeedDirection:
    """Crop the speed and direction commands to the extremum values."""

    speed = cmd_vel.speed
    direction = cmd_vel.direction

    # Crop the speed command
    if speed > speed_lim["max"]:
        speed = speed_lim["max"]
    elif speed < speed_lim["min"]:
        speed = speed_lim["min"]

    # Crop the direction command
    if direction > direction_lim["max"]:
        direction = direction_lim["max"]
    elif direction < direction_lim["min"]:
        direction = direction_lim["min"]

    return SpeedDirection(speed, direction)



def nav_3_dials(lidar_data:LaserScan, Kspeed:float, Kdir:float, Karg:float, Mode:str="spaced", FrontRatio: float = 0.2, **args) -> SpeedDirection:
    """Return the speed and direction of the robot based on 3 dials range data."""

    # Get the ranges of the dials
    if Mode == "spaced" :
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
    arg = (np.argmax(lidar_data.ranges) - mid_i)/mid_i

    rospy.loginfo(f"arg : {arg}")
    # Compute the speed command
    speed_cmd = Kspeed * dist_center

    # Compute the direction command
    dir_cmd = (- Kdir * (dist_right - dist_left) / dist_center /0.2) + (Karg * arg)

    cmd_vel = SpeedDirection(speed_cmd, dir_cmd)

    # Crop the commands
    return crop_cmd_vel(cmd_vel, speed_lim={"min":0.2, "max":1})


def nav_n_dials(lidar_data:LaserScan, Kspeed:float, Kdir:float, Karg:float, n_dials:int, Mode:str, FrontRatio:float = 0.2, **args) -> SpeedDirection:
    """Return the speed and direction of the robot based on 3 dials range data."""

    # Get the ranges of the dials
    range_left, range_right = get_dials_ranges(lidar_data, n_dials=2)
    _, range_center, _ = get_dials_ranges(lidar_data, n_dials=3, proportion=[1 - FrontRatio, FrontRatio * 2, 1 - FrontRatio])

    # Compute the mean distance of each dial
    dist_left = np.mean(range_left)
    dist_center = np.mean(range_center)
    dist_right = np.mean(range_right)

    # Argmax step
    mid_i = len(lidar_data.ranges)//2

    rospy.loginfo("Not implemented yet, doing a simple argmax")
    arg = (np.argmax(lidar_data.ranges) - mid_i)/mid_i # à changer avec le bon algorithme

    # Compute the speed command
    speed_cmd = Kspeed * dist_center

    # Compute the direction command

    if "classic" in Mode :
        dir_cmd = - Kdir * (dist_right - dist_left) / dist_center / 0.1 + Karg * arg

    elif "pondéré" in Mode :
        dir_cmd = (- Kdir * (dist_right - dist_left) / dist_center / 0.1+ Karg * arg) / (Kdir + Karg)

    elif "division" in Mode : 
        dir_cmd = - dist_right/dist_left - 1 if dist_right > dist_left else dist_left/dist_right - 1
        dir_cmd = (Kdir * dir_cmd / dist_center / 0.1 + Karg * arg) / (Kdir + Karg)

    cmd_vel = SpeedDirection(speed_cmd, dir_cmd)

    # Crop the commands
    return crop_cmd_vel(cmd_vel, speed_lim={"min":0.2, "max":1})


def backward_with_color_turn(camera_info:CameraInfo, green_is_left:bool, backward_speed:float=-1.0, turn_magnitude:float=0.8, **args) -> SpeedDirection:
    """Return the speed and direction commands to go backward and turn to the color side."""

    if backward_speed > 0:
        rospy.logwarn("The backward speed must be negative. Here backward_speed = {}".format(backward_speed))
    if turn_magnitude < 0 or turn_magnitude > 1:
        rospy.logwarn("The turn magnitude must be between 0 and 1. Here turn_magnitude = {}".format(turn_magnitude))


    speed = backward_speed
    direction = 0

    # If the robot is in front of a color, go backward turning to the color side
    if camera_info.front_color in ["red", "green"]:
        direction = turn_magnitude * (1 if (camera_info.front_color == "red" and green_is_left)  else -1)
    
    return crop_cmd_vel(SpeedDirection(speed, direction), speed_lim={"min":-1, "max":0}, direction_lim={"min":-1, "max":1})