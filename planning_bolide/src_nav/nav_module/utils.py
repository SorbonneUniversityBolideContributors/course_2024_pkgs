import rospy
from sensor_msgs.msg import LaserScan
from control_bolide.msg import SpeedDirection
import numpy as np


__author__ = "Eliot CHRISTON"
__status__ = "Development"
__version__ = "1.0.2"


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

    return np.array(dials, dtype=object)


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