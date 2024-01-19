#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Raphael KHORASSANI"
__status__ = ""
__version__ = ""
__annotations__ = ""

import numpy as np
import cv2
import rospy

from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import Bool


class DetectColor:
    def __init__(self) :
        self.color = rospy.get_param("/color_to_calibrate", default = "no_one")
        if self.color != "no_one" :
            rospy.Subscriber("raw_image_data", SensorImage, self.callback_image)

    def callback_image(self, image_data) :
        im = np.array(image_data.data)
        H,W,c = im.shape
        wH = int(H * 0.2)
        wW = int(W * 0.4)

        zone_of_interest = im[(W - wW)//2 : (W-wW)//2 + wW, (H - wH)//2 : (H - wH)//2 + wH]

        values = np.median(zone_of_interest, axis = 2)
        min_threshold = values - 50
        max_threshold = values + 50

        min_threshold[min_threshold < 0] = 0
        min_threshold[min_threshold > 255] = 255
        max_threshold[max_threshold < 0] = 0
        max_threshold[max_threshold > 255] = 255

        if self.color == "red" :
            rospy.set_param("/red_threshold", [list(min_threshold), list(max_threshold)])
        elif self.color == "green" :
            rospy.set_param("/green_threshold", [list(min_threshold), list(max_threshold)])

        rospy.set_param("/color_to_calibrate", "no_one")


if __name__ == "main" :
    detect_color = DetectColor()
