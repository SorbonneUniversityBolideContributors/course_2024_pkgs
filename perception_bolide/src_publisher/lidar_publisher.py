#!/usr/bin/env python3

__author__ = "Maxime Chalumeau"
__email__ = "maxime.chalumeau@etu.sorbonne-universite.fr"
__status__ = "Development"
__version__ = "1.2.1"

import rospy
from rplidar import RPLidar, RPLidarException
from sensor_msgs.msg import LaserScan
import time
import threading
from numpy import pi

class LidarController:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('lidar_publisher')

        # variables
        self.publish_rate = 12 # in Hz
        self.min_range = 0.17 # in m
        self.max_range = 12 # in m
        self.scan_time = 1/self.publish_rate # in s
        self.time_increment = self.scan_time/360 # in s (because there are 360 points)
        
        # Connect to the RPLidar
        self.lidar = RPLidar("/dev/ttyUSB0", baudrate=256000)
        self.lidar.connect()

        try:
            # Log the info from the lidar
            rospy.loginfo(f"[INFO] -- Lidar connected {self.lidar.get_info()}")
        except Exception as e:
            # Log an error if we can't connect to the lidar
            rospy.logerr(f"[ERROR] -- {e}\nerror, can't connect to Lidar")
            exit(1)

        # Start the lidar motor
        rospy.loginfo(f"[INFO] -- Starting Lidar")
        self.lidar.start_motor()
        time.sleep(1) # wait for the motor to start

        # Initialize the publisher node
        self.pub = rospy.Publisher('raw_lidar_data', LaserScan, queue_size=10)
        
        # set all the data that are fixed in LidarScan
        self.lidar_data = LaserScan()
        self.lidar_data.header.frame_id = "lidar_frame"
        self.lidar_data.angle_min = 0 # in radians
        self.lidar_data.angle_max = 359*pi/180 # in radians
        self.lidar_data.angle_increment = pi / 180 # in radians (should be 1 degree)
        self.lidar_data.scan_time = self.scan_time # in seconds
        self.lidar_data.time_increment = self.time_increment # in seconds
        self.lidar_data.range_min = self.min_range # in meters
        self.lidar_data.range_max = self.max_range # in meters

        # We use a thread because when using rospy.Rate in the same method where we collecting the data, it creates rates issues
        # Create and start a separate thread for collecting lidar data
        self.collect_thread = threading.Thread(target=self.collect_data)
        self.collect_thread.start()

        # Create an Event object for signaling the thread to stop
        self.stop_thread = threading.Event()

        # Create a lock for thread-safe access to self.lidar_data
        self.data_lock = threading.Lock()

        # Start publishing lidar data
        time.sleep(1) # wait for the first data to be collected
        rospy.loginfo(f"[INFO] -- Start publishing on topic /raw_lidar_data")
        self.start_publish()

    def collect_data(self):
        # Continuously fill the array with lidar scan data
        for scan in self.lidar.iter_scans(scan_type='express') :
            # Create an array of 360 zeros
            array_lidar = [0]*360
            # Print the number of points in the scan for debug
            rospy.logdebug(f"nb pts : {len(scan)}")
            # Store the scan data in the array
            for i in range(len(scan)) :
                angle = min(359,max(0,359-int(scan[i][1]))) #scan[i][1]:angle
                array_lidar[angle]=scan[i][2] / 1000 #scan[i][2]:distance in mm -> m

            # Lock the varaible to avoid publishing with wrong time stamp
            with self.data_lock:
                # Add time stamp and data to the LidarScan object
                self.lidar_data.header.stamp = rospy.Time.now()
                self.lidar_data.ranges = array_lidar

            # Check the stop signal after each scan
            if self.stop_thread.is_set():
                return

    def start_publish(self):
        rate = rospy.Rate(self.publish_rate)  # Define the rate object with your desired rate in Hz

        try:
            rate = rospy.Rate(self.publish_rate)
            while not rospy.is_shutdown():
                # Publish data in lidar_data
                self.pub.publish(self.lidar_data)
                rate.sleep()

        except (RPLidarException, ValueError, rospy.ROSInterruptException) as e:
            rospy.loginfo(f"an error occured: {e}")
        finally:
            # If an exception occurs (mainly because of ROSInterruptException)
            # Signal the thread to stop
            self.stop_thread.set()
            # Wait for the thread to finish
            self.collect_thread.join()

            # shut down the lidar
            rospy.loginfo("[INFO] -- shutting down lidar...")
            self.lidar.stop_motor()
            self.lidar.stop()
            time.sleep(1)
            self.lidar.disconnect()
            rospy.loginfo("[INFO] -- lidar shutted down")
            exit(0)

if __name__ == '__main__':
    try:
        # Create a LidarController and start it
        lidarcontrol = LidarController()
    except rospy.ROSInterruptException:
        # If a ROSInterruptException occurs, exit the program
        exit(0)
