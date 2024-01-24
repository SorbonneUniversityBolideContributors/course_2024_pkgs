#!/usr/bin/env python3

__author__ = "Eliot CHRISTON"
__status__ = "Development"
__version__ = "1.0.0"

#%% IMPORTS
import rospy
from control_bolide.msg import SpeedDirection
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from perception_bolide.msg import MultipleRange
from perception_bolide.msg import CameraInfo


#%% CLASS
class NavSensors():
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
        self.lidar_data = None
        self.camera_info = None
        self.rear_range_data = None

        # Parameters
        # TODO

    def get_params(self, value = True):
        # TODO
        pass

    def callback_lidar(self, data):
        self.lidar_data = data
    
    def callback_camera(self, data):
        self.camera_info = data
    
    def callback_multiple_range(self, data):
        self.rear_range_data = data
    
    def set_cmd_vel(self):
        # TODO
        pass


#%% MAIN
if __name__ == '__main__' :
    my_nav = NavSensors()
    rospy.spin()