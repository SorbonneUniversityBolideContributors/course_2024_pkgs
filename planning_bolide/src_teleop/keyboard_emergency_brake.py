#!/usr/bin/env python3

__author__ = "Loris OUMBICHE"
__status__ = "Tested"

import numpy as np
import rospy
import sys
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener  #used for reading the keyboard

class Keyboard_ctrl():
    def __init__(self):
        self.pub = rospy.Publisher("emergency_brake", Bool, queue_size=1)
        self.space_pressed = False
        self.last_space_pressed = False 

    def on_press(self, key):
        if key == Key.space:
            self.last_space_pressed = self.space_pressed #get the last state of the space key
            self.space_pressed = not self.space_pressed  #toggle the space_pressed variable on and off

    '''def on_release(self, key):  # if somehow you want to brake while you hold the space key
         Remove the following lines to change the behavior
         if key == Key.space:
              self.last_space_pressed = self.space_pressed
              self.space_pressed = False'''

    def get_space_pressed(self): #publish the space_pressed variable if it has changed
        if self.space_pressed != self.last_space_pressed:
            self.pub.publish(self.space_pressed)
            self.last_space_pressed = self.space_pressed


def register_keyboard_listener(s: Keyboard_ctrl): 
    listener = Listener(on_press=s.on_press)  # Register the listener to the keyboard
    listener.start()
    return listener
    
def listener_keyboard(s: Keyboard_ctrl):
    rate = rospy.Rate(10)  # Adjust the rate as needed
    listener = register_keyboard_listener(s)

    while not rospy.is_shutdown(): 
        s.get_space_pressed()
        rate.sleep()

    listener.stop()


if __name__ == '__main__':
    rospy.init_node('keyboard_emergency_brake')
    s = Keyboard_ctrl()

    try:
        listener_keyboard(s)

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("Shutting down...")
        sys.exit()
