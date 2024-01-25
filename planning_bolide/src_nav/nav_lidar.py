#!/usr/bin/env python3

__author__ = "Loris OUMBICHE"
__status__ = "Development"
__version__ = "3.0.0"


#%% IMPORTS
import rospy
import sys
from control_bolide.msg import SpeedDirection
from sensor_msgs.msg import LaserScan
from nav_module.nav_functions import nav_3_dials, nav_n_dials_with_biases

#%% CLASS
class NavLidar():
    def __init__(self):
        print("NavLidar init")
        self.get_all_params()
        self.pub = rospy.Publisher("cmd_vel",SpeedDirection,queue_size=1)
        self.cmd = SpeedDirection()
    
    def get_scan(self, msg:LaserScan):
        if self.Mode == "3Dials_classic" :
            self.cmd = nav_3_dials(msg, self.Kv, self.Kd, self.Ka)
        else :
            self.cmd = nav_n_dials_with_biases(msg, self.Kv, self.Kd, self.Ka, self.n_dials, self.Mode)
        self.pub.publish(self.cmd)

    def get_all_params(self, value = True):
        # Gains
        self.Kd = rospy.get_param("/gain_direction", default = 0.8)
        self.Kv = rospy.get_param("/gain_vitesse", default = 0.33)
        self.Ka = rospy.get_param("/gain_direction_arg_max", default = 0.2)

        self.Mode = rospy.get_param("/navigation_mode", default = "3Dials_classic")
        self.feature = rospy.get_param("/navigation_feature", default = "mean")

        self.n_dials = rospy.get_param("/navigation_n_dials", default = 11)

def listener(s:NavLidar):
    rospy.Subscriber('lidar_data',LaserScan,s.get_scan)
    rospy.spin()


if __name__ == '__main__' :
    rospy.init_node('Nav_LIDAR')
    s = NavLidar()

    try : 
        listener(s)
    except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit()
 

