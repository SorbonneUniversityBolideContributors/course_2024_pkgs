#!/usr/bin/env python3

__author__ = "Eliot CHRISTON"
__status__ = "Development"
__version__ = "1.2.5"

#%% IMPORTS
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

import numpy as np

from control_bolide.msg import SpeedDirection
from perception_bolide.msg import MultipleRange
from perception_bolide.msg import CameraInfo

from nav_module.forward_functions import nav_3_dials, nav_n_dials
from nav_module.backward_functions import backward_with_color_turn
from nav_module.utils import get_dials_ranges


#%% CLASS
class NavSensors():
    """Class that handles the navigation of the robot.
    It directly uses the sensor data to determine the speed and direction of the robot."""

    def __init__(self):
        # Initialize the node
        rospy.init_node("nav_3states", anonymous = True)

        # Subscribe to the sensor data
        rospy.Subscriber("lidar_data", LaserScan, self.callback_lidar)
        rospy.Subscriber("camera_info", CameraInfo, self.callback_camera_info)
        rospy.Subscriber("raw_rear_range_data", MultipleRange, self.callback_multiple_range)

        rospy.Subscriber("/param_change_alert", Bool, self.get_params)

        # init the publisher
        self.pub = rospy.Publisher("cmd_vel", SpeedDirection, queue_size=10)

        # Initialize the stored data
        self.lidar_data = LaserScan()
        self.camera_info = CameraInfo()
        self.rear_range_data = MultipleRange()

        # conditions
        self.wrong_way = False              # True if the robot is going the wrong way
        self.front_too_close = False        # True if the front of the robot is too close
        self.rear_too_close = False         # True if the rear of the robot is too close
        self.front_far_enough = True        # True if the front of the robot is far enough

        # stored values
        self.previous_state = "forward"      # The previous state of the robot
        self.current_state = "forward"       # The current state of the robot
        self.protocol_entry = ("forward", "backward")    # The entry of the protocol, generally the previous and current state when different. This is used to be sure that the protocol follows the good state transition.
        self.cmd_vel = SpeedDirection()     # The command to publish
        self.navigation_dict = {            # The navigation functions (see nav_functions.py)
            "3Dials":nav_3_dials,
            "NDials":nav_n_dials
        }
        self.nav_features = {               # The features to use for the navigation (generally the statistics of the dials)
            "mean": np.mean,
            "max": np.max,
            "min": np.min,
            "median": np.median,
            "q1" : lambda x: np.percentile(x, 25),
            "q3" : lambda x: np.percentile(x, 75),
        }

        self.get_params()

# PARAMS UPDATE ===============================================================
    def get_params(self, value = True):
        """Update the parameters when the parameter change alert is received."""
        rospy.loginfo("Updating the parameters")

        # Gains
        self.Kd = rospy.get_param("/gain_direction", default = 0.3)
        self.Kv = rospy.get_param("/gain_vitesse", default = 0.33)
        self.Ka = rospy.get_param("/gain_direction_arg_max", default = 0.2)

        # Race direction
        self.green_is_left = rospy.get_param("/green_is_left", default = True)

        # Thresholds for changing state
        self.threshold_front_too_close = rospy.get_param("/threshold_front_too_close", default = 0.2)
        self.threshold_rear_too_close = rospy.get_param("/threshold_rear_too_close", default = 0.2)
        self.threshold_front_far_enough = rospy.get_param("/threshold_front_far_enough", default = 0.5)

        # Forward navigation mode
        navigation_mode = rospy.get_param("/navigation_mode", default = "3Dials_classic")
        self.nav_func, self.nav_mode = navigation_mode.split("_")
        self.nav_feature_choice = rospy.get_param("/navigation_feature", default = "median")
        self.nav_is_spaced = rospy.get_param("/spaced_dials", default = True)
        self.front_dial_ratio = rospy.get_param("/front_dial_ratio", default = 0.2)

        # Maximize threshold
        self.use_maximize_threshold = rospy.get_param("/use_maximize_threshold", default = False)
        self.maximize_threshold = rospy.get_param("/maximize_threshold", default = 0.5)

# PROTOCOLS ===================================================================
    def protocol_through_neutral(self):
        """Protocol to go through the neutral point."""
        self.pub.publish(SpeedDirection(0, 0))
        rospy.sleep(0.15)
    
    def protocol_brake(self):
        """Protocol to brake."""
        self.pub.publish(SpeedDirection(2, 0))
        rospy.sleep(0.15)

    def protocol_inverse_prop(self):
        """Protocol to go to backward from forward."""
        self.protocol_brake()
        self.protocol_through_neutral()
    
    def apply_protocol(self):
        """Apply the protocol to go to the next state."""
        transitions = {
            ("forward"  , "backward"): self.protocol_inverse_prop,
            ("forward"  , "stop")    : None,
            ("backward", "forward")  : self.protocol_through_neutral,
            ("backward", "stop")    : None,
            ("stop"    , "forward")  : None,
            ("stop"    , "backward"): self.protocol_through_neutral,
        }
        protocol = transitions[self.protocol_entry]
        if protocol is not None:
            protocol()

# CALLBACKS ===================================================================
    def callback_lidar(self, data):
        self.lidar_data = data
        self.nav_step()
    
    def callback_camera_info(self, data):
        self.camera_info = data
        self.nav_step() # be careful, this is a fast callback
    
    def callback_multiple_range(self, data):
        self.rear_range_data = data
        self.nav_step()

# PUBLISHERS ==================================================================
    def publish_cmd_vel(self):
        self.pub.publish(self.cmd_vel)

# CONDITIONS ==================================================================
    def update_conditions(self):
        """Update the conditions of the robot."""
        
        # compute the current distances (THIS SHOULD BE DONE IN A PROCESSING NODE)
        _, front_ranges, _ = get_dials_ranges(self.lidar_data, n_dials=3, proportion=[1, 0.5, 1])
        current_front_distance = np.percentile(front_ranges, 25)
        current_rear_distance = np.min([self.rear_range_data.IR_rear_left.range, self.rear_range_data.IR_rear_right.range])

        # update conditions
        self.front_too_close  = current_front_distance < self.threshold_front_too_close
        self.front_far_enough = current_front_distance > self.threshold_front_far_enough
        self.rear_too_close   = current_rear_distance  < self.threshold_rear_too_close
    
# STATES ======================================================================
    def forward_state(self):
        """Update the speed and direction when the robot is going forward."""
        # Get the navigation function
        navigation_function = self.navigation_dict[self.nav_func]

        # Get the command
        self.cmd_vel = navigation_function(
            lidar_data=self.lidar_data,
            Kspeed=self.Kv,
            Kdir=self.Kd,
            Karg=self.Ka,
            mode=self.nav_mode,
            is_spaced=self.nav_is_spaced,
            navigation_feature=self.nav_features[self.nav_feature_choice],
            FrontRatio = self.front_dial_ratio,
            use_maximise_threshold = self.use_maximize_threshold,
            maximise_threshold = self.maximize_threshold
        )

    def backward_state(self):
        """Update the speed and direction when the robot is going backward."""
        self.cmd_vel = backward_with_color_turn(self.camera_info, self.green_is_left, backward_speed=-0.7)

    def stop_state(self):
        """Update the speed and direction when the robot is stopped."""
        speed = 0 if self.previous_state == "backward" else 2
        self.cmd_vel = SpeedDirection(speed, 0)

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
                self.current_state = "forward"
            elif self.front_too_close and self.rear_too_close:
                self.current_state = "stop"
        else: # forward or stop
            if not self.front_too_close:
                self.current_state = "forward"
            elif not self.rear_too_close: # and self.front_too_close
                self.current_state = "backward"
            else: # self.rear_too_close and self.front_too_close
                self.current_state = "stop"


# MAIN LOOP ===================================================================
    def nav_step(self):
        """Main loop step of the navigation."""

        self.update_conditions()

        self.previous_state = self.current_state
        self.next_state()

        if self.previous_state != self.current_state:
            self.protocol_entry = (self.previous_state, self.current_state)
            rospy.loginfo("From {} to {}".format(self.previous_state, self.current_state))
            rospy.loginfo("   RTC = {}, FTC = {}, FFE = {}".format(self.rear_too_close, self.front_too_close, self.front_far_enough))
            self.apply_protocol()
        
        state_actions = {
            "forward"  : self.forward_state,
            "backward": self.backward_state,
            "stop"    : self.stop_state,
        }
        state_actions[self.current_state]()
        
        self.publish_cmd_vel()
        

#%% MAIN
if __name__ == '__main__' :
    my_nav = NavSensors()
    rospy.spin()