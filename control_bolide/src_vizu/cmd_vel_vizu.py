#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from control_bolide.msg import SpeedDirection

class CmdVelVisualisation :
	def __init__(self) :
		rospy.Subscriber("cmd_vel", SpeedDirection, self.callback)
		rospy.spin()

	def callback(self, data) :	
		cmd_vel = data.speed
		cmd_dir = data.direction

		s = 200
		im = np.zeros([s,s, 3], dtype = 'uint8')
		im = cv2.arrowedLine(im, (s//2, s//2), (s//2,int((1 - cmd_vel) * s//2)),(0,150,255), 5)
		im = cv2.arrowedLine(im, (s//2, s//2), (int((1 - cmd_dir) * s//2),s//2),(255,150,0), 5)

		cv2.imshow('im', im)
		cv2.waitKey(1)


if __name__ == '__main__':

	rospy.init_node('cmd_vel_vizu', anonymous = True)
	
	navigation = CmdVelVisualisation()