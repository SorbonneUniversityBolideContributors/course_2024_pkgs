import rospy
from perception_bolide.msg import CameraInfo
from control_bolide.msg import SpeedDirection
from nav_module.utils import crop_cmd_vel

__author__ = "Eliot CHRISTON"
__status__ = "Development"
__version__ = "1.0.0"


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
        direction = turn_magnitude * (1 if ((camera_info.front_color == "red" and green_is_left) or (camera_info.front_color == "green" and not green_is_left)) else -1)
    
    return crop_cmd_vel(SpeedDirection(speed, direction), speed_lim={"min":-1, "max":0}, direction_lim={"min":-1, "max":1})