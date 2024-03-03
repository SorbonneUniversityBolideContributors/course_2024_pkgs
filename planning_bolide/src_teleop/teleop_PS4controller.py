#!/usr/bin/env python3

__author__ = "Maxime Chalumeau"
__email__ = "maxime.chalumeau@etu.sorbonne-universite.fr"
__status__ = "Tested"

import rospy
from pyPS4Controller.controller import Controller
from control_bolide.msg import SpeedDirection
import signal

# Translate controller triggers input [-32768, 32767] into motor output values [0, 1]
def transf_trigger(raw):
    temp = (raw+32767)/65534
    # round the value
    return round(temp, 3)

# Translate controller triggers input ([-32767,0] when left [0, 32767] when right) into motor output values [-1, 1]
def transf_joystick(raw):
    temp = (raw/32767)
    # round the value
    return round(temp, 3)

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

        # current value of speed and direction (because of watchdog)
        self.current_speed = 0
        self.current_direction = 0

        # use to make the first pression on trigger brake and second one backward
        self.first_press_L2 = True
        self.braking = False
        
        # 
        self.NEUTRAL = 0
        self.BRAKE = 2

        # 
        self.CENTER = 0

        # init publisher node
        self.pub = rospy.Publisher('cmd_vel', SpeedDirection, queue_size=10)

        # init timer for publishing
        self.timer = rospy.Timer(rospy.Duration(0.4), self.timer_callback)

    def timer_callback(self, event):
        self.publish_speed_direction()

    def on_R2_press(self, value):
        if not self.braking:
            self.first_press_L2 = True
            value = transf_trigger(value)
            rospy.loginfo(f"R2 value: {value}")
            self.current_speed = value
            # publish value on /cmd_vel
            self.publish_speed_direction()

    def on_R2_release(self):
        if not self.braking:
            rospy.loginfo("R2 released")
            self.current_speed = 0
            # publish speed=0 on /cmd_vel
            self.publish_speed_direction()

    def on_L2_press(self, value):
        if self.first_press_L2:
            self.braking = True
            self.current_speed = self.BRAKE
            rospy.loginfo(f"braking")
        else:
            value = -transf_trigger(value)
            rospy.loginfo(f"L2 value: {value}")
            self.current_speed = value
        # publish value on /cmd_vel
        self.publish_speed_direction()

    def on_L2_release(self):
        self.braking = False
        self.first_press_L2 = False
        rospy.loginfo("L2 released")
        self.current_speed = 0
        # publish speed=0 on /cmd_vel
        self.publish_speed_direction()

    def on_L3_x_at_rest(self):
        rospy.loginfo(f"straight")
        self.current_direction = 0
        # publish direction=0 on /cmd_vel
        self.publish_speed_direction()

    def on_L3_right(self, value):
        rospy.loginfo(f"right value: {value}")
        value = transf_joystick(value)
        self.current_direction = value
        # publish direction=value on /cmd_vel
        self.publish_speed_direction()

    def on_L3_left(self, value):
        rospy.loginfo(f"left value: {value}")
        value = transf_joystick(value)
        self.current_direction = value
        # publish direction=value on /cmd_vel
        self.publish_speed_direction()

    def publish_speed_direction(self):
        msg = SpeedDirection()
        msg.speed = self.current_speed
        msg.direction = self.current_direction
        self.pub.publish(msg)
    
    def on_options_press(self):
        self.current_speed = 0
        self.current_direction = 0
        self.publish_speed_direction()
        self.timer.shutdown()  # stop the timer
        exit(0)

    def on_disconnect(self):
        self.current_speed = 0
        self.current_direction = 0
        self.publish_speed_direction()
        self.timer.shutdown()  # stop the timer
        super.on_disconnect()


if __name__ == '__main__':
    controller = None
    def signal_handler(sig, frame):
        if controller is not None:
            controller.on_options_press()
        exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    try:
        rospy.init_node('teleop_node')
        controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
        controller.listen()
    except rospy.ROSInterruptException:
        pass