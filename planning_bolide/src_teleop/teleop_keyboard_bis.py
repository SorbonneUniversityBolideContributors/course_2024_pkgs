#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys, termios, tty
from control_bolide.msg import SpeedDirection


import click

# Arrow keys codes
keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}

msg=SpeedDirection()

speed = 0
direction = 0

def publish_msg(event):
    pub.publish(msg)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        pub = rospy.Publisher("cmd_vel", SpeedDirection, queue_size=10)
        rospy.init_node("teleop_keyboard", anonymous=True)
        rate = rospy.Rate(300)
        timer = rospy.Timer(rospy.Duration(0.4), publish_msg)
        pub.publish(msg)
        msg=SpeedDirection()
        msg.speed= speed
        msg.direction = direction
        try:    
            # Get character from console
            mykey = click.getchar()
            if mykey in keys.keys():
                char=keys[mykey]

            if char == 'up':    # UP key
                print("up")
                if speed == 2.0:
                    speed = 0.0
                speed += 0.1

                
                # Do something
            elif char == 'down':  # DOWN key
                print("down")
                if speed == 2.0:
                    speed = 0.0
                speed += -0.1
                # Do something
            elif char == 'left':  # RIGHT key
                direction -= 0.8
                # Do something
            elif char == 'right': # LEFT
                direction += 0.8
                # Do something
            elif char == 'stop':
                speed = 2.0
            elif char == "quit":  # QUIT
                rospy.signal_shutdown("Quit")
                # Do something
            rate.sleep()
        except rospy.ROSInterruptException:
            pub.publish(msg)