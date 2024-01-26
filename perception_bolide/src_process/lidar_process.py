#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Raphael KHORASSANI and Eliot CHRISTON"
__status__ = "Tested"
__version__ = "3.3.0"
__annotations__ = "Based on Maxime C.'s work (version 1.0.0)"

#%% IMPORTS

import numpy as np
import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

#%% ProcessLidarData class
class ProcessLidarData:
    """Class used to process the lidar data and publish it on a new topic"""

    def __init__(self):
        # Initialize the ROS node
        rospy.loginfo("[INFO] -- Initializing the lidar process data node")
        rospy.init_node('lidar_process_data_node')

        # Store last arrays containing lidar data
        self.last_values = []

        # Set the parameters :
        self.get_parameters()

        # SUBSCRIBER ========================================
        rospy.Subscriber("raw_lidar_data", LaserScan, self.callback)

        rospy.Subscriber("/param_change_alert", Bool, self.callback_parameters)
        # PUBLISHER =========================================
        self.pub = rospy.Publisher('lidar_data', LaserScan, queue_size=10)

        rospy.spin() # wait for the callback to be called


    def get_parameters(self):

        # Set the filter parameters
        self.temporal_filter           = rospy.get_param('/temporal_filter_bool'    , default = False)
        self.spatial_filter            = rospy.get_param('/spatial_filter_bool'     , default = False)
        self.anti_jumping_filter       = rospy.get_param('/anti_jumping_filter_bool', default = False)
        self.spatial_filter_range      = rospy.get_param('/spatial_filter_range'    , default = 1)
        self.temporal_filter_range     = rospy.get_param('/temporal_filter_range'   , default = 5)
        self.anti_jumping_filter_range = rospy.get_param('/anti_jumping_filter_range', default = 5)

        # all the following parameters could be set in the launch file
        self.min_angle_deg          = rospy.get_param("/lidar_min_angle_deg", -90) # in degrees
        self.max_angle_deg          = rospy.get_param("/lidar_max_angle_deg", 90) 
        self.min_angle_rad          = self.min_angle_deg * np.pi / 180 # in radians
        self.max_angle_rad          = self.max_angle_deg * np.pi / 180 # in radians

    def callback_parameters(self, data:Bool) :

        # When we change the range of angles, it can cause problem for temporal median filtering
        # We must then reinitialize the last values stored
        old_min, old_max = self.min_angle_deg, self.max_angle_deg
        self.get_parameters()

        if old_min != self.min_angle_deg or old_max != self.max_angle_deg :
            self.last_values = []

        try : self.iii += 1
        except : self.iii = 1
        rospy.logdebug(f"[DEBUG] -- Retrieved parameters for the {self.iii}th time")

    def callback(self, data:LaserScan) :
        """ Callback function called when a message is received on the subscribed topic"""

        # Check that the data array has a length of 360
        if not (len(data.ranges) == 360):
            rospy.logdebug("the lidar array is not composed of 360 values")
            return

        # Check that the min angle is less than the max angle
        if not self.min_angle_deg < self.max_angle_deg:
            rospy.logwarn("The min angle must be inferior to the max angle")
            return

        # Check that the min and max angles are integers
        if not (isinstance(self.min_angle_deg, int) and isinstance(self.max_angle_deg, int)):
            rospy.logerr("The min and max angles must be integers")
            return

        # crop the data
        cropped_data = self.crop_data(data.ranges)

        # apply filters to data
        data_filtered = self.filter_lidar(cropped_data)
        
        # retrieving data from the LidarSensor msg
        lidar_data = LaserScan()
        lidar_data.header.stamp = rospy.Time.now()
        lidar_data.header.frame_id = "lidar_frame"
        lidar_data.angle_min = self.min_angle_rad # in radians
        lidar_data.angle_max = self.max_angle_rad # in radians
        lidar_data.angle_increment = data.angle_increment # in radians (should be 1 degree)
        lidar_data.time_increment = data.time_increment # in seconds
        lidar_data.scan_time = data.scan_time # in seconds
        lidar_data.range_min = data.range_min # in meters
        lidar_data.range_max = data.range_max # in meters

        lidar_data.ranges = data_filtered

        self.pub.publish(lidar_data)

    def crop_data(self, data:list) :
        """ Crop the data array to keep only the data between min_angle and max_angle """

        # Convert the data list to a numpy array
        data_array = np.array(data)

        # Rotate the data array so that the min angle is at the start
        data_array = np.roll(data_array, -self.min_angle_deg)

        # Crop the data array to keep only the data between min_angle and max_angle
        data_array = data_array[:self.max_angle_deg - self.min_angle_deg]

        # Check that the cropped data array has the correct length
        if not (len(data_array) == self.max_angle_deg - self.min_angle_deg):
            rospy.logwarn("The cropped data array must have a length of max_angle - min_angle")
            return

        # Return the cropped data array as a list
        return list(data_array)


    def filter_lidar(self, data:list):
        """Filter the lidar data using a median filter"""

        # Convert the data list to a numpy array
        data_array = np.array(data)

        if self.spatial_filter :
            # Spatial median filter:
            # Copy the original data array
            final_array = np.copy(data_array)

            # For each value in the spatial filter range, shift the data array by that amount in both directions,
            # stack the shifted arrays on top of the final array
            for i in range(1, self.spatial_filter_range + 1) :
                final_array = np.vstack([final_array, np.roll(data_array, i)])
                final_array = np.vstack([final_array, np.roll(data_array, -i)])

            # Replace the original data array with the median of the final array
            data_array = np.median(final_array, axis = 0)

        if self.temporal_filter :
            # Temporal median filter:
            # Add the current data array to the list of last values
            self.last_values += [data_array]

            # If the list of last values is longer than the temporal filter range, remove the oldest values
            while len(self.last_values) > self.temporal_filter_range : self.last_values.pop(0)

            # Convert the list of last values to a numpy array
            data_array = np.array(self.last_values)

            # Replace the original data array with the median of the last values
            data_array = np.median(data_array, axis = 0)
        
        if self.anti_jumping_filter :
            # Anti-jumping filter to prevent unexpected jumps in the lidar data:
            # For each null value in the data array, replace it with the median of the values around it (non-null values) (around in a temporal sense)
            self.last_values += [data_array]

            # If the list of last values is longer than the anti-jumping filter range, remove the oldest values
            while len(self.last_values) > self.anti_jumping_filter_range : self.last_values.pop(0)

            # Convert the list of last values to a numpy array
            data_array = np.array(self.last_values)

            # Replace the null values with the median of the last values at the same index
            data_array = np.where(data_array == 0, np.median(data_array, axis = 0), data_array)

        # Return the filtered data array as a list
        return list(data_array)



#%% Main
if __name__ == '__main__':
    try:
        # Create a ProcessLidarData and start it
        lidarprocess = ProcessLidarData()
    except rospy.ROSInterruptException:
        # If a ROSInterruptException occurs, exit the program
        exit(0)

