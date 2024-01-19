#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Raphael KHORASSANI"
__status__ = ""
__version__ = ""
__annotations__ = ""

import numpy as np
import cv2
import rospy

from perception_bolide.msg import CameraInfo
from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import Bool


class CameraProcess:
    def __init__(self) :
        rospy.Subscriber("raw_image_data", SensorImage, self.callback_image)
        rospy.Subscriber("/param_change_alert", Bool, self.get_params)

        self.get_params()

        self.pub = rospy.Publisher("camera_info", CameraInfo, queue_size = 10)

        rospy.spin()
    
    def callback_image(self, image_data) :
        im = np.array(image_data.data)
        H,W,c = im.shape
        w = W//2

        green_mask  = cv2.inRange(im, self.green_threshold[0], self.green_threshold[1])
        red_mask    = cv2.inRange(im, self.red_threshold[0], self.red_threshold[1])

        green_side  = int(self.green_side + 1) % 2
        red_side    = (green_side + 1) % 2

        green_split = green_mask[w*green_side : (w+1)*green_side, 0:H]
        red_split   = red_mask[w*red_side : (w+1)*red_side, 0:H]

        camera_info_msg = CameraInfo()
        camera_info_msg.wrong_way = False
        camera_info_msg.front_color = "ok"

        self.pub.publish(camera_info_msg)

    def get_params(self, value = True):
        self.red_threshold = rospy.get_param("/red_threshold", default = [[200,0,0],[255,100,100]])
        self.green_threshold = rospy.get_param("/green_threshold", default = [[0,200,0],[100,255,100]])
        self.green_side = rospy.get_param("green_is_left", default = True)

if __name__ == "__main__" :
    camera_process = CameraProcess()
