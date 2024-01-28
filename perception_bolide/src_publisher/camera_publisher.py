#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Maxime Chalumeau"
__email__ = "maxime.chalumeau@etu.sorbonne-universite.fr"
__status__ = "Tested"
__version__ = "1.0.0"

import rospy
import time
import cv2
from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import Bool

class CameraPublisher :

	def __init__(self, w, h, fr) :
		# Initialize camera parameters
		self.w, self.h, self.fr = w, h, fr
		
		rospy.Subscriber("/param_change_alert", Bool, self.get_enable_camera)
	
		self.get_enable_camera()
		# Log camera initialization
		rospy.loginfo("[INFO] -- Camera init. : {}x{}px -- Frame rate : {}".format(self.w, self.h, self.fr))

		# Set up camera capture
		self.camera = cv2.VideoCapture(0)
		self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
		self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
		self.camera.set(cv2.CAP_PROP_FPS, self.fr)
		time.sleep(0.1)

		# Log camera initialization completion
		rospy.loginfo("[INFO] -- Camera initialisation done.")

		# Initialize image data for publishing
		self.image_data = SensorImage()
		self.image_data.encoding = "rgb8"
		self.image_data.is_bigendian = False
		self.image_data.height, self.image_data.width = self.h, self.w
		self.image_data.step = 3 * self.w # This is the full row length in bytes. It's 3 times the width because each pixel has three color channels (RGB).

		# Set up publisher for image data
		self.image_pub  = rospy.Publisher("raw_image_data", SensorImage, queue_size=1)

	def publish_scan(self) :
		""" Publishing image array in ROS topic at rate : self.fr """
		# Log start of image publishing
		rospy.loginfo("[INFO] -- Images are published in topic : /raw_image_data")

		# Continuously capture and publish images until ROS is shutdown
		while not rospy.is_shutdown():
			if self.enable_camera :
				ret, frame = self.camera.read()
				if not ret:
					break
				frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
				self.image_data.header.stamp = rospy.Time.now()
				self.image_data.data = frame.tobytes()

				# Publish the image
				self.image_pub.publish(self.image_data)

		# Log end of node
		rospy.loginfo("[INFO] -- Stopping node")

	def get_enable_camera(self, value = True) :
		self.enable_camera = rospy.get_param("/enable_camera_bool", default = True)

if __name__ == "__main__" :
	# Initialize ROS node
	rospy.init_node("camera_publisher")

	# Get parameters for image dimensions and frame rate
	width  = rospy.get_param("image_width", default=160)
	height = rospy.get_param("image_height", default=128)
	framerate = rospy.get_param("frame_rate", default=20)

	# Initialize and run CameraPublisher
	cam_pub = CameraPublisher(width, height, framerate)
	cam_pub.publish_scan()
