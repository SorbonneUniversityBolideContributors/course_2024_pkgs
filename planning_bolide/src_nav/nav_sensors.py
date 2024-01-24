#!/usr/bin/env python3

__author__ = "Eliot CHRISTON"
__status__ = "Development"
__version__ = "1.0.0"

#%% IMPORTS
import rospy
import numpy as np
from control_bolide.msg import SpeedDirection
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from perception_bolide.msg import MultipleRange
from perception_bolide.msg import CameraInfo
import nav_functions as nf


#%% CLASS
class NavSensors():
    """Class that handles the navigation of the robot.
    It directly uses the sensor data to determine the speed and direction of the robot."""

    def __init__(self):
        # Initialize the node
        rospy.init_node("nav_sensors", anonymous = True)

        # Subscribe to the sensor data
        rospy.Subscriber("lidar_data", LaserScan, self.callback_lidar)
        rospy.Subscriber("camera_info", CameraInfo, self.callback_camera)
        rospy.Subscriber("rear_range_data", MultipleRange, self.callback_multiple_range)

        rospy.Subscriber("param_change_alert", Bool, self.get_params)

        # init the publisher
        self.pub = rospy.Publisher("cmd_vel", SpeedDirection, queue_size=10)

        # Initialize the stored attributes
        self.lidar_data = LaserScan()
        self.camera_info = CameraInfo()
        self.rear_range_data = MultipleRange()

        # conditions
        self.wrong_way = False
        self.front_too_close = False
        self.rear_too_close = False
        self.front_far_enough = True

        # current values
        self.current_state = "foward"
        self.cmd_vel = SpeedDirection()

        # Parameters
        self.min_front_distance = 0.2 # The minimum distance in front of the robot
        self.min_rear_distance = 0.1 # The minimum distance behind the robot
        self.far_enough_front_distance = 0.5 # The distance in front of the robot to consider it is far enough to go forward
        self.Kv = 0.5 # The speed coefficient
        self.Kd = 0.5 # The direction coefficient
        self.Ka = 0.5 # The argmax coefficient
        self.green_is_left = True # True if the green side is on the left of the robot

        self.get_params()


# PARAMS UPDATE ===============================================================
    def get_params(self, value = True):
        """Update the parameters when the parameter change alert is received."""
        self.Kd = rospy.get_param("/gain_direction", default = 0.8)
        self.Kv = rospy.get_param("/gain_vitesse", default = 0.33)
        self.Ka = rospy.get_param("/gain_direction_arg_max", default = 0.2)
        self.green_is_left = rospy.get_param("/green_is_left", default = True)

# CALLBACKS ===================================================================
    def callback_lidar(self, data):
        self.lidar_data = data
    
    def callback_camera(self, data):
        self.camera_info = data
    
    def callback_multiple_range(self, data):
        self.rear_range_data = data
    
    def set_cmd_vel(self):
        self.pub.publish(SpeedDirection(self.speed, self.direction))
    
# STATES ======================================================================
    def foward_state(self):
        """Update the speed and direction when the robot is going forward."""
        self.cmd_vel = nf.nav_3_dials(self.lidar_data, self.Kv, self.Kd, self.Ka)

    def backward_state(self):
        """Update the speed and direction when the robot is going backward."""
        self.cmd_vel = nf.backward_with_color_turn(self.camera_info, self.green_is_left)

    def stop_state(self):
        """Update the speed and direction when the robot is stopped."""
        # TODO
        pass

    def next_state(self):
        """Update the current state of the robot.
        
        FTC = Front Too Close
        RTC = Rear Too Close
        FFE = Front Far Enough

                    |      CURRENT TO NEXT STATE     |
        FTC RTC FFE | stop     | forward  | backward |
        ----------------------------------------------
        0   0   0   | forward  | forward  | backward |
        0   0   1   | forward  | forward  | forward  |
        0   1   0   | forward  | forward  | forward  |
        0   1   1   | forward  | forward  | forward  |
        1   0   0   | backward | backward | backward |
        1   0   1   |-----------IMPOSSIBLE-----------|
        1   1   0   | stop     | stop     | stop     |
        1   1   1   |-----------IMPOSSIBLE-----------|
        """
        if self.current_state == "backward":
            if not self.front_too_close and (self.rear_too_close or self.front_far_enough):
                self.current_state = "foward"
            elif self.front_too_close and self.rear_too_close:
                self.current_state = "stop"
        else: # forward or stop
            if not self.front_too_close:
                self.current_state = "foward"
            elif not self.rear_too_close: # and self.front_too_close
                self.current_state = "backward"
            else: # self.rear_too_close and self.front_too_close
                self.current_state = "stop"
        

#%% MAIN
if __name__ == '__main__' :
    my_nav = NavSensors()
    rospy.spin()