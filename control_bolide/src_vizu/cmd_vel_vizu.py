#!/usr/bin/env python3
# -*- coding: utf-8 -*-
__author__ = "Raphael KHORASSANI"
__status__ = ""
__version__ = ""
__annotations__ = ""

import numpy as np
import cv2
import rospy
from control_bolide.msg import SpeedDirection

class CmdVelVisualisation :
	def __init__(self, mode = 2) :
		if mode == 1 : 
			rospy.Subscriber("cmd_vel", SpeedDirection, self.callback)
		elif mode == 2 :
			self.init_compteur()
			rospy.Subscriber("cmd_vel", SpeedDirection, self.callback_compteur)
		elif mode == 3 : 
			self.init_written()
			rospy.Subscriber("cmd_vel", SpeedDirection, self.callback_written)
		rospy.spin()

	def init_written(self) :
		w = 500
		h = (w*2) // 2
		im = np.zeros([h,w,3], dtype = np.uint8)

		jauge_w = int(w * 0.7)
		xi_jauge = (w-jauge_w)//2
		im[xi_jauge : xi_jauge + jauge_w, int(h*0.35), int(h*0.45)] = [150]*3
		# im = cv2.putText(im, )

	
	def callback_written(self, data):
		vitesse = data.speed
		
		i = int(vitesse)
		j = int((vitesse - i) * self.decimals)
		angle = np.pi * (1 + (i * self.decimals + j)/(self.maxi*self.decimals))
		x = int(s*r * 0.85 * np.cos(angle) + s//2)
		y = int(s*r * 0.85 * np.sin(angle) + s)
		im = self.im.copy()
		im = cv2.arrowedLine(im,(s//2,s),(x,y), [255]*3,2 )
		im = cv2.circle(im, (s//2, s), int(s*r*0.3), (0,0,255), cv2.FILLED)
		im = im[s//2:s, 0:s]
		cv2.imshow("Compteur Vitesse", im)
		cv2.waitKey(1)


	def callback(self, data) :	
		cmd_vel = data.speed
		cmd_dir = data.direction

		s = 200
		im = np.zeros([s,s, 3], dtype = 'uint8')
		im = cv2.arrowedLine(im, (s//2, s//2), (s//2,int((1 - cmd_vel) * s//2)),(0,150,255), 5)
		im = cv2.arrowedLine(im, (s//2, s//2), (int((1 - cmd_dir) * s//2),s//2),(255,150,0), 5)

		cv2.imshow('im', im)
		cv2.waitKey(1)
		
	def init_compteur(self) :
		self.s = 1000
		self.im = np.zeros((2*self.s,self.s,3), dtype = np.uint8)
		self.r = 3/4/2
		self.maxi = 38
		self.im = cv2.circle(self.im, (self.s//2, self.s), int(self.s*self.r), [255]*3, 2)
		self.decimals = 3
		for i in range(self.maxi) :
			for j in range(self.decimals)  :
				angle = np.pi * (1 + (i * self.decimals + j)/(self.maxi*self.decimals))
				x = self.s*self.r * np.cos(angle) + self.s//2
				y = self.s*self.r * np.sin(angle) + self.s
				thickness = 2 if j == 0 else 1
				self.im = cv2.line(self.im, (int(x), int(y)), (self.s//2, self.s), [255]*3, thickness)
			
		self.im = cv2.circle(self.im, (self.s//2, self.s), int(self.s*self.r*0.9), 0, cv2.FILLED)
		
	def callback_compteur(self, data) :
		vitesse = data.speed
		
		s,r = self.s, self.r
		i = int(vitesse)
		j = int((vitesse - i) * self.decimals)
		angle = np.pi * (1 + (i * self.decimals + j)/(self.maxi*self.decimals))
		x = int(s*r * 0.85 * np.cos(angle) + s//2)
		y = int(s*r * 0.85 * np.sin(angle) + s)
		im = self.im.copy()
		im = cv2.arrowedLine(im,(s//2,s),(x,y), [255]*3,2 )
		im = cv2.circle(im, (s//2, s), int(s*r*0.3), (0,0,255), cv2.FILLED)
		im = im[s//2:s, 0:s]
		cv2.imshow("Compteur Vitesse", im)
		cv2.waitKey(1)


if __name__ == '__main__':

	rospy.init_node('cmd_vel_vizu', anonymous = True)
	
	navigation = CmdVelVisualisation()
