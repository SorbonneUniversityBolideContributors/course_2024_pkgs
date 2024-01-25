#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Eliot CHRISTON"
__status__ = "Development"
__version__ = "2.0.0"
__annotations__ = "Based on Raphael K.'s work"

import numpy as np
import cv2
import rospy

from perception_bolide.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


def rgb2hsv(rgb:tuple) -> tuple:
    """Convert the RGB color to HSV."""
    return cv2.cvtColor(np.uint8([[rgb]]), cv2.COLOR_RGB2HSV)[0][0]


class CameraProcess:
    def __init__(self) :
        
        # Initialize the node
        rospy.init_node("camera_info", anonymous = True)

        # log info
        rospy.loginfo("Initializing the camera_info node")

        # Subscribe to the image data
        rospy.Subscriber("raw_image_data", Image, self.callback_image)

        # Subscribe to the parameter change alert
        rospy.Subscriber("/param_change_alert", Bool, self.get_ROS_params)

        self.get_ROS_params()

        # Initialize the stored attributes
        self.image_matrix = None # The image matrix with HSV values
        self.wrong_way = False # True if the robot is going the wrong way

        # Parameters
        self.mid_range = 3 # The range around the middle pixel to get the mean pixel
        self.side_range = 3 # The range around the side pixel to get the mean pixel

        self.pub = rospy.Publisher("camera_info", CameraInfo, queue_size = 10)

        rospy.spin()

    def callback_image(self, image_data:Image) :
        """Callback function for the image data subscriber."""
        # Convert the image to a numpy array reshaped to the image size
        self.image_matrix = np.frombuffer(image_data.data, dtype = np.uint8)
        self.image_matrix = self.image_matrix.reshape(image_data.height, image_data.width, 3)

        middle_color, left_color, right_color = self.middle_and_side_colors()

        self.is_wrong_way(left_color, right_color)

        # Build the CameraInfo message
        camera_info_msg = CameraInfo()
        camera_info_msg.wrong_way = self.wrong_way
        camera_info_msg.front_color = middle_color
        camera_info_msg.left_color = left_color
        camera_info_msg.right_color = right_color

        # Publish the message
        self.pub.publish(camera_info_msg)

    def nearest_color(self, pixel:np.ndarray) -> str:
        """Return the nearest color of the pixel if the difference is under a tolerance threshold. Else return "unknown"."""
        hsv_pixel = rgb2hsv(pixel)
        max_dist = np.linalg.norm(np.array([0, 0, 0]) - np.array([180, 255, 255]))
        differences = {color : np.linalg.norm(hsv_pixel - self.HSV_COLORS[color]) / max_dist for color in self.HSV_COLORS}

        print(differences)

        if min(differences.values()) < self.tolerance:
            return min(differences, key = lambda x : differences[x])
        else:
            return "unknown"

    def is_wrong_way(self, left_color:str, right_color:str) -> None:
        """Store if the robot is going the wrong way in the self.wrong_way variable."""

        if left_color == "green" and right_color == "red":
            # If the green is on the left and the red on the right, the robot is going "green is left"
            self.wrong_way = not self.green_is_left
        elif left_color == "red" and right_color == "green":
            # If the red is on the left and the green on the right, the robot is going "green is right"
            self.wrong_way = self.green_is_left
        else:
            # The robot can't know if it is going the wrong way, so it doesn't change its state
            pass
    
    def middle_and_side_colors(self) -> tuple:
        """Return the color of the middle pixel of the image."""
        if self.image_matrix is None:
            return "white"
        
        # Get the middle index of the image
        mid_index = self.image_matrix.shape[0]//2, self.image_matrix.shape[1]//2

        # Get the mean pixel around the middle pixel
        middle_pixel_mean = np.mean(
            self.image_matrix[(mid_index[0] - self.mid_range) : (mid_index[0] + self.mid_range), (mid_index[1] - self.mid_range) : (mid_index[1] + self.mid_range)],
            axis = (0,1)
        )
        # Get the left mean pixel around the horizontal left and vertical middle pixel
        left_pixel_mean = np.mean(
            self.image_matrix[(mid_index[0] - self.side_range) : (mid_index[0] + self.side_range), :self.side_range],
            axis = (0,1)
        )
        
        # Get the right mean pixel around the horizontal right and vertical middle pixel
        right_pixel_mean = np.mean(
            self.image_matrix[(mid_index[0] - self.side_range) : (mid_index[0] + self.side_range), -self.side_range:],
            axis = (0,1)
        )

        return self.nearest_color(middle_pixel_mean), self.nearest_color(left_pixel_mean), self.nearest_color(right_pixel_mean)

    def get_ROS_params(self, value = True):
        """Update the parameters when the parameter change alert is received."""
        self.RGB_COLORS = {
            "red"    : rospy.get_param("/red_RBG", default = (255, 0, 0)),
            "green"  : rospy.get_param("/green_RGB", default = (0, 255, 0)),
        }
        self.HSV_COLORS = {color : rgb2hsv(self.RGB_COLORS[color]) for color in self.RGB_COLORS}

        self.tolerance = rospy.get_param("/color_detection_tolerance", default = 0.25) # The tolerance percentage for the color detection


if __name__ == "__main__" :
    camera_process = CameraProcess()
