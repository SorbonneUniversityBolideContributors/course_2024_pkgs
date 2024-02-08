#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import necessary libraries
import numpy as np
import rospy

# Import ROS message types
from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import Bool

# Author information
__author__ = "Raphael KHORASSANI"
__status__ = "Development"

# Define a class for color detection and calibration
class DetectColor:
    def __init__(self):
        # Initialize ROS publisher for calibration status
        self.pub = rospy.Publisher("/is_auto_calibration_done", Bool, queue_size=10)
        # Initialize the listener
        self.listener_init()

    # Initialize the ROS subscriber to know when we request an auto calibration
    def listener_init(self):
        rospy.Subscriber("/do_an_auto_calibration", Bool, self.listener)
        rospy.spin()

    # This is a listener but also a callback : 
    #   - When we request an auto calibration, the node will call this callback
    #   - It will subscribe to the topic /raw_image_data until it receives one SensorImage message from this topic
    #   - Then it will unsubscribe to this topic as we did the calibration.
    def listener(self, value=True):
        # Get the color to calibrate from ROS parameter server
        self.color = rospy.get_param("/color_to_calibrate", default="no_one")
        if self.color != "no_one":
            # Subscribe to the image topic
            self.subscriber = rospy.Subscriber("raw_image_data", SensorImage, self.callback_image)
        else:
            rospy.loginfo("Color is no_one")

    # Callback function for image processing
    def callback_image(self, image_data):

        # Extract image data and reshape
        h, w = image_data.height, image_data.width
        im = np.frombuffer(image_data.data, dtype=np.uint8)
        im = np.reshape(im, (h, w, 3))

        # Define zone of interest
        H, W, c = im.shape
        wH = int(H * 0.2)
        wW = int(W * 0.4)
        zone_of_interest = im[(W - wW) // 2 : (W - wW) // 2 + wW, (H - wH) // 2 : (H - wH) // 2 + wH]

        # Calculate median of the zone of interest
        values = np.median(zone_of_interest, axis=(0, 1))

        # Set parameter name based on color
        param_name = f"/{self.color}_RGB"

        # Set parameters on ROS parameter server
        rospy.set_param(param_name, values.astype(np.uint8).tolist())
        # When we finished, we want to set the parameter /color_to_calibrate to "no_one" as a security, so that the node cannot do a calibration if we don't precise the color we want to calibrate.
        rospy.set_param("/color_to_calibrate", "no_one")

        # Log calibration completion
        rospy.loginfo(f"Calibration done for {self.color}")

        # Publish calibration status, to tell to the MainWindow to refresh it's values
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)

        # Unregister the subscriber, to not subscribe unnecessarily to the topic /raw_image_data
        self.subscriber.unregister()

# Initialize the ROS node and start the color calibration
rospy.init_node("calibrate_color", anonymous=False)
detect_color = DetectColor()
