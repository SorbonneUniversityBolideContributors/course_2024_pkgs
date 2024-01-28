#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Raphael KHORASSANI and Eliot CHRISTON and Loris OUMBICHE"
__status__ = "Tested"
__version__ = "3.2.3"
__annotations__ = "Based on Maxime C.'s work"


#%% IMPORTS
import matplotlib.pyplot as plt
import numpy as np
import rospy
import sys 

from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import Bool

#%% LidarVizu class

class LidarVizu:
	"""Class used to plot the lidar data"""

	def __init__(self, rmax:int=7): 
		
		# Initialize the figure
		self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
		self.ln_raw_data, = plt.plot([], [], '.b', label="raw data")
		self.ln_processed_data, = plt.plot([], [], '.r', label="processed data")
		plt.legend(loc="upper left")

		self.get_rmax()
		
		self.data_raw, self.data_processed = [0]*360, [0]*360
		self.angles = np.linspace(0, 2*np.pi, 360)
	
	def initPlot(self) -> list:
		"""Initialize the plot"""
		# set the limits of the polar plot
		# offset
		self.ax.set_theta_offset(np.pi/2.0)
		self.ax.set_rmax(self.rmax)
		self.ax.set_title("Lidar Plot")
		self.ax.grid(color="gray")
		self.ax.set_facecolor((0.0, 0.0, 0.0))

		# set labels with white color
		self.ax.tick_params(axis='y', colors='white')

		return [self.ln_raw_data, self.ln_processed_data]
		
	def callback_raw_data(self, msg:LaserScan):
		"""Callback function for the raw lidar data"""
		self.data_raw = msg.ranges

		assert len(self.data_raw) == 360, "The raw lidar data must be 360 points long, not {}".format(len(self.data_raw))
		
	def callback_processed_data(self, msg:LaserScan):
		"""Callback function for the processed lidar data"""
		scan = msg.ranges
		min_angle = int(180 * msg.angle_min / np.pi)
		
		self.data_processed = [0]*360
		for i in range(len(scan)):
			index = (i + min_angle) % 360
			self.data_processed[index] = scan[i]

	def get_rmax(self, value = True) :
		self.rmax = rospy.get_param("/lidar_rmax", default = 5000) 	# in QT the unit of the slider is mm
		self.ax.set_rmax(self.rmax)

	def update_plot(self, frame):
		"""Update the plot"""
		print(self.angles[:10])
		print(self.data_raw[:10])
		print(self.data_processed[:10])
		print()
		self.ln_raw_data.set_data(self.angles, self.data_raw)
		self.ln_processed_data.set_data(self.angles, self.data_processed)
		return [self.ln_raw_data, self.ln_processed_data]
			
			
#%% LISTENER FUNCTION
	
def listener(plot:LidarVizu, host:str=''):
	"""Listen to the lidar data and plot it"""
	rospy.Subscriber("raw_lidar_data", LaserScan, plot.callback_raw_data)
	rospy.Subscriber("lidar_data", LaserScan, plot.callback_processed_data)
	rospy.Subscriber("/param_change_alert", Bool, plot.get_rmax)

	plt.show(block = True)
	

#%% MAIN
if __name__ == '__main__':

	rospy.init_node('lidar_vizu', anonymous = True)
	
	my_plot = LidarVizu() 
	
	my_animation = FuncAnimation(my_plot.fig, my_plot.update_plot, init_func = my_plot.initPlot)

	try:
		listener(my_plot)
	except (rospy.ROSInterruptException, KeyboardInterrupt):
		sys.quit() 
		

		 
