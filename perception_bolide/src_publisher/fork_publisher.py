#!/usr/bin/env python3

__author__ = "Quentin Rolland"
__email__ = "quentin.rolland@ensea.fr"
__status__ = "Development"
__version__ = "1.0.0"

import rospy
from perception_bolide.msg import ForkSpeed
from std_msgs.msg import Float32MultiArray

class Optical_Fork:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('fork_publisher')

        # SUBSCRIBER ========================================
        rospy.Subscriber("stm32_sensors", Float32MultiArray, self.callback)
        # PUBLISHER =========================================
        self.pub = rospy.Publisher('raw_fork_data', ForkSpeed, queue_size=10)

        rospy.spin() # wait for the callback to be called

    def callback(self, data:Float32MultiArray) :
        """ Callback function called when a message is received on the subscribed topic"""
        
        # retrieving Fork data from the STM32_sensors msg
        fork_data = ForkSpeed()
        fork_data.speed = data.data[2] # speed of the car in mm/s

        # Process fork data
        fork_data.speed = fork_data.speed/1000 # passage de mm/s à m/s

        fork_data.header.stamp = rospy.Time.now()
        self.pub.publish(fork_data)
        

if __name__ == '__main__':
    try:
        # Create an Optical_Fork and start it
        fork_data = Optical_Fork()
    except rospy.ROSInterruptException:
        # If a ROSInterruptException occurs, exit the program
        exit(0)