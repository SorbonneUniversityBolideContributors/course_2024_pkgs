#!/usr/bin/env python3

__author__ = "Quentin Rolland"
__email__ = "quentin.rolland@ensea.fr"
__status__ = "Development"
__version__ = "1.0.0"

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

class IMU_data:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('imu_publisher')

        # SUBSCRIBER ========================================
        rospy.Subscriber("STM32_sensors_topic", Float32MultiArray, self.callback)
        # PUBLISHER =========================================
        self.pub = rospy.Publisher('raw_imu_data', Imu, queue_size=10)

        rospy.spin() # wait for the callback to be called

    def callback(self, data:Float32MultiArray) :
        """ Callback function called when a message is received on the subscribed topic"""
        
        # retrieving IMU data from the STM32_sensors msg
        imu_data = Imu()
        imu_data.orientation.z = data.data[0] # not the Quaternion but the Euler_yaw
        imu_data.linear_acceleration.x = data.data[1] # x acceleration

        # Process IMU data
        imu_data.orientation.z = imu_data.orientation.z/900 # conversion in radian
        imu_data.linear_acceleration.x = imu_data.linear_acceleration.x/100 # conversion en m/s^2


        imu_data.header.stamp = rospy.Time.now()
        self.pub.publish(imu_data)
        

if __name__ == '__main__':
    try:
        # Create an Optical_Fork and start it
        imu_raw = IMU_data()
    except rospy.ROSInterruptException:
        # If a ROSInterruptException occurs, exit the program
        exit(0)