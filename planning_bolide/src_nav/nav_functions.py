
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

    print("dials_indices:", dials_indices)

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

def nav_3_dials(lidar_data:LaserScan, Kspeed:float, Kdir:float, Karg:float) -> SpeedDirection:
    """Return the speed and direction of the robot based on 3 dials range data."""

    # Get the ranges of the dials
    range_left, _, range_center, _, range_right = get_dials_ranges(lidar_data, n_dials=5, proportion=[7, 1, 2, 1, 7])

    # Compute the mean distance of each dial
    dist_left = np.mean(range_left)
    dist_center = np.mean(range_center)
    dist_right = np.mean(range_right)

    # Argmax step
    mid_i = len(lidar_data.ranges)//2
    arg = (np.argmax(lidar_data.ranges) - mid_i)/mid_i

    # Compute the speed command
    speed_cmd = Kspeed * dist_center

    # Compute the direction command
    dir_cmd = - Kdir * (dist_right - dist_left) / speed_cmd + Karg * arg

    cmd_vel = SpeedDirection(speed_cmd, dir_cmd)

    # Crop the commands
    return crop_cmd_vel(cmd_vel, speed_lim={"min":0.2, "max":1})


def backward_with_color_turn(camera_info:CameraInfo, max_backward_speed:float=1.0) -> SpeedDirection:
    """Return the speed and direction commands to go backward and turn to the color side."""

    speed = -max_backward_speed

    if camera_info.front_color in ["red", "green"]:
        # If the robot is in front of a color, go backward turning to the color side
        pass