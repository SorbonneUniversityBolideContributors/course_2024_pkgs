#!/usr/bin/env python3

__author__ = "Maxime Chalumeau"
__email__ = "maxime.chalumeau@etu.sorbonne-universite.fr"
__status__ = "Development"

import rospy
from control_bolide.msg import SpeedDirection
from std_msgs.msg import Bool
from rpi_hardware_pwm import HardwarePWM
import time

class ControllerListener:
    def __init__(self):
        # Max speed authorized
        self.MAXSPEED = 8.2 # percentage cyclic ratio
        self.MINSPEED = 7.95
        self.NEUTRAL = 7.5  # 7.5 = neutral
        self.REVERSEMINSPEED = 7
        self.REVERSEMAXSPEED = 6.5
        self.BRAKE = 5

        # Current speed
        self.breaking = False

        # Values for the direction
        self.CENTER = 6.95
        self.LEFT = 5.5
        self.RIGHT = 8.4
        self.left_range = abs(self.CENTER - self.LEFT)
        self.right_range = abs(self.CENTER - self.RIGHT)

        # init propulsion pwm and direction pwm
        self.pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
        self.pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
        self.pwm_dir.start(self.CENTER)
        self.pwm_prop.start(self.NEUTRAL)

        # Node initialisation
        rospy.init_node('speed_direction_listener')
        rospy.Subscriber("cmd_vel", SpeedDirection, self.callback) # Subscribe to cmd_vel for velocities and directions command
        rospy.Subscriber("emergency_break", Bool, self.emergency_break_callback) # Subscribe to emergency_break for emergency break command

        # Initialize watchdog timer
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.5), self.watchdog_callback)
        self.last_command_time = rospy.get_time()

        # initialise state machine
        self.sm = StateMachine(self)

        rospy.spin()

    def callback(self, data):
        # if emergency break is activated, don't do anything
        if self.emergency_break:
            return

        # get speed and direction from the message
        speed = data.speed
        direction = data.direction

        # crop speed between -1 and 1 (except if speed == 2 it's used for breaking)
        if speed != 2:
            if speed < -1: speed = -1
            elif speed > 1: speed = 1
        # crop direction between -1 and 1
        if direction <-1: direction = -1
        elif direction > 1: direction = 1

        # change the states of the State machine
        if speed == 0:
            if self.sm.state == "Break":
                self.sm.neutral_after_break()
            else:
                self.sm.neutral_after_forward()
        elif 1 >= speed > 0:
            self.sm.forward(speed)
        elif speed < 0:
            self.sm.backward(speed)
        elif speed == 2:
            self.sm.break_()

        # change the direction of the robot
        if direction < 0:
            self.pwm_dir.change_duty_cycle(self.CENTER+direction*self.left_range)
        else:
            self.pwm_dir.change_duty_cycle(self.CENTER+direction*self.right_range)

        # Update the last command time
        self.last_command_time = rospy.get_time()

    # callback for the emergency break command
    def emergency_break_callback(self, data):
        # if the emergency break is activated, stop the robot
        if data.data:
            self.emergency_break = True
            if self.sm.state in ["Backward", "Neutral_After_Break"]:
                # if the robot is moving backward, just go neutral
                # sending a break command when going backward will make it move backward faster
                self.pwm_prop.change_duty_cycle(self.NEUTRAL)
            else:
                # else, send a break command
                self.pwm_prop.change_duty_cycle(self.BRAKE)
        else:
            self.emergency_break = False

    def watchdog_callback(self, event):
        # If it's been more than 0.5 seconds since the last command, stop the robot
        # This is to prevent the robot from moving if the controller or computer crashes or disconnects
        if (rospy.get_time() - self.last_command_time > 0.5) and not self.emergency_break:
            if self.sm.state == "Break":
                self.sm.neutral_after_break()
            else:
                self.sm.neutral_after_forward()

# State machine class
class StateMachine:
    """
    State machine for the robot

    States:
    - Forward: Moving forward
    - Backward: Moving backward
    - Neutral_After_Forward: Stopped moving after moving forward
    - Neutral_After_Break: Stopped moving after breaking or moving backward
    - Break: Break applied

    Transitions:
    - Forward: Neutral_After_Forward, Break, Neutral_After_Break, Forward
    - Backward: Neutral_After_Break, Backward
    - Neutral_After_Forward: Forward, Neutral_After_Forward
    - Neutral_After_Break: Backward, Break, Neutral_After_Break
    - Break: Forward, Neutral_After_Forward, Break
    """

    def __init__(self, controller: ControllerListener):
        self.controller = controller
        self.state = "Neutral_After_Forward"

    def forward(self, speed):
        if self.state in ["Neutral_After_Forward", "Break", "Neutral_After_Break", "Forward"]:
            if self.state != "Forward": rospy.loginfo("Moving forward"); self.state = "Forward"
            self.controller.pwm_prop.change_duty_cycle(self.controller.MINSPEED + speed*(self.controller.MAXSPEED-self.controller.MINSPEED))
        else:
            rospy.logwarn(f"Can't transition to Forward from {self.state}")

    def backward(self, speed):
        if self.state in ["Neutral_After_Break","Backward"]:
            if self.state != "Backward": rospy.loginfo("Moving backward"); self.state = "Backward"
            self.controller.pwm_prop.change_duty_cycle(self.controller.REVERSEMINSPEED + speed*(self.controller.REVERSEMINSPEED-self.controller.REVERSEMAXSPEED))
        else:
            rospy.logwarn(f"Can't transition to Backward from {self.state}")

    def neutral_after_forward(self):
        if self.state in ["Forward", "Neutral_After_Forward"]:
            if self.state != "Neutral_After_Forward": rospy.loginfo("Stopped moving after moving forward"); self.state = "Neutral_After_Forward"
            self.controller.pwm_prop.change_duty_cycle(self.controller.NEUTRAL)
        else:
            rospy.logwarn(f"Can't transition to Neutral_After_Forward from {self.state}")

    def neutral_after_break(self):
        if self.state in ["Backward", "Break", "Neutral_After_Break"]:
            if self.state != "Neutral_After_Break": rospy.loginfo("Stopped moving after breaking or moving backward"); self.state = "Neutral_After_Break"
            self.controller.pwm_prop.change_duty_cycle(self.controller.NEUTRAL)  
        else:
            rospy.logwarn(f"Can't transition to Neutral_After_Break from {self.state}")

    def break_(self):
        if self.state in ["Forward", "Neutral_After_Forward", "Break"]:
            if self.state != "Break": rospy.loginfo("Break applied"); self.state = "Break"
            self.controller.pwm_prop.change_duty_cycle(self.controller.BRAKE)
        else:
            rospy.logwarn(f"Can't transition to Break from {self.state}")

if __name__ == '__main__':
    try:
        listener = ControllerListener()
    except rospy.ROSInterruptException:
        pass
