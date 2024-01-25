#!/usr/bin/env python3

__author__ = "Eliot CHRISTON"
__status__ = "Development"
__version__ = "1.2.0"

#%% IMPORTS
import rospy
import numpy as np
from control_bolide.msg import SpeedDirection
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from perception_bolide.msg import MultipleRange
from perception_bolide.msg import CameraInfo
from nav_module.nav_functions import nav_3_dials, backward_with_color_turn, get_dials_ranges


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

        rospy.Subscriber("param_change_alert", Bool, self.get_params)

        # init the publisher
        self.pub = rospy.Publisher("cmd_vel", SpeedDirection, queue_size=10)

        # Initialize the stored data
        self.lidar_data = LaserScan()
        self.camera_info = CameraInfo()
        self.rear_range_data = MultipleRange()

        # conditions
        self.wrong_way = False
        self.front_too_close = False
        self.rear_too_close = False
        self.front_far_enough = True

        # stored values
        self.previous_state = "foward"
        self.current_state = "foward"
        self.cmd_vel = SpeedDirection()

        # Parameters
        self.threshold_front_too_close = 0.2 # The minimum distance in front of the robot
        self.threshold_rear_too_close = 0.1 # The minimum distance behind the robot
        self.threshold_front_far_enough = 0.5 # The distance in front of the robot to consider it is far enough to go forward
        self.Kv = 0.5 # The speed coefficient
        self.Kd = 0.5 # The direction coefficient
        self.Ka = 0.5 # The argmax coefficient
        self.green_is_left = True # True if the green side is on the left of the robot

        self.get_params()

# PARAMS UPDATE ===============================================================
    def get_params(self, value = True):
        """Update the parameters when the parameter change alert is received."""
        self.Kd = rospy.get_param("/gain_direction", default = 0.3)
        self.Kv = rospy.get_param("/gain_vitesse", default = 0.33)
        self.Ka = rospy.get_param("/gain_direction_arg_max", default = 0.2)

        self.green_is_left = rospy.get_param("/green_is_left", default = True)

        self.threshold_front_too_close = rospy.get_param("/threshold_front_too_close", default = 0.2)
        self.threshold_rear_too_close = rospy.get_param("/threshold_rear_too_close", default = 0.1)
        self.threshold_front_far_enough = rospy.get_param("/threshold_front_far_enough", default = 0.5)

# PROTOCOLS ===================================================================
    def protocol_through_neutral(self):
        """Protocol to go through the neutral point."""
        self.pub.publish(SpeedDirection(0, 0))
        rospy.sleep(0.05)
    
    def protocol_brake(self):
        """Protocol to brake."""
        self.pub.publish(SpeedDirection(2, 0))
        rospy.sleep(0.1)

    def protocol_inverse_prop(self):
        """Protocol to go to backward from forward."""
        self.protocol_brake()
        self.protocol_through_neutral()
    
    def apply_protocol(self):
        """Apply the protocol to go to the next state."""
        transitions = {
            ("foward"  , "backward"): self.protocol_inverse_prop,
            ("foward"  , "stop")    : None,
            ("backward", "foward")  : self.protocol_through_neutral,
            ("backward", "stop")    : None,
            ("stop"    , "foward")  : None,
            ("stop"    , "backward"): self.protocol_through_neutral,
        }
        protocol = transitions[(self.previous_state, self.current_state)]
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
        _, front_ranges, _ = get_dials_ranges(self.lidar_data, n_dials=3, proportion=[1, 0.5, 1])
        current_front_distance = np.mean(front_ranges)
        self.front_too_close = current_front_distance < self.threshold_front_too_close
        self.front_far_enough = current_front_distance > self.threshold_front_far_enough

        current_rear_distance = np.min([self.rear_range_data.IR_rear_left.range, self.rear_range_data.IR_rear_right.range])
        print("rear left: ", self.rear_range_data.IR_rear_left.range)
        print("rear right: ", self.rear_range_data.IR_rear_right.range)
        self.rear_too_close = current_rear_distance < self.threshold_rear_too_close
    
# STATES ======================================================================
    def foward_state(self):
        """Update the speed and direction when the robot is going forward."""
        self.cmd_vel = nav_3_dials(self.lidar_data, self.Kv, self.Kd, self.Ka)

    def backward_state(self):
        """Update the speed and direction when the robot is going backward."""
        self.cmd_vel = backward_with_color_turn(self.camera_info, self.green_is_left)

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


# MAIN LOOP ===================================================================
    def nav_step(self):
        """Main loop step of the navigation."""

        self.update_conditions()

        self.previous_state = self.current_state
        self.next_state()

        print("time: ", rospy.get_time())
        print("   state: ", self.previous_state)
        print("   front_too_close: ", self.front_too_close)
        print("   rear_too_close: ", self.rear_too_close)
        print("   front_far_enough: ", self.front_far_enough)

        if self.previous_state != self.current_state:
            self.apply_protocol()
        
        state_actions = {
            "foward"  : self.foward_state,
            "backward": self.backward_state,
            "stop"    : self.stop_state,
        }
        state_actions[self.current_state]()
        
        self.publish_cmd_vel()
        

#%% MAIN
if __name__ == '__main__' :
    my_nav = NavSensors()
    rospy.spin()