#!/usr/bin/env python3

__author__ = "Loris OUMBICHE"
__status__ = "Development"
__version__ = "3.0.0"


#%% IMPORTS
import rospy
import sys
from control_bolide.msg import SpeedDirection
from sensor_msgs.msg import LaserScan
from nav_module.nav_functions import nav_3_dials, nav_3_dials_spaced, nav_n_dials

#%% CLASS
class NavLidar():
    def __init__(self):
        print("NavLidar init")
        self.get_all_params()
        self.pub = rospy.Publisher("cmd_vel",SpeedDirection,queue_size=1)
        self.cmd = SpeedDirection()
        self.navigation_dict = {
            "3Dials_classic":nav_3_dials,
            "3Dials_spaced":nav_3_dials_spaced,
            "nDials":nav_n_dials
        }
        self.navigation_choice = "3Dials_spaced"
    
    def get_scan(self, msg:LaserScan):
        navigation_function = self.navigation_dict[self.navigation_choice]
        self.cmd = navigation_function(
            lidar_data=msg,
            Kspeed=self.Kv,
            Kdir=self.Kd,
            Karg=self.Ka,
            Mode=self.Mode,
            n_dials=self.n_dials
        )
        self.pub.publish(self.cmd)

    def get_all_params(self, value = True):
        # Gains
        self.Kd = rospy.get_param("/gain_direction", default = 0.8)
        self.Kv = rospy.get_param("/gain_vitesse", default = 0.33)
        self.Ka = rospy.get_param("/gain_direction_arg_max", default = 0.2)

        self.Mode = rospy.get_param("/navigation_mode", default = "3Dials_classic")
        self.feature = rospy.get_param("/navigation_feature", default = "mean")

        self.use_dials = rospy.get_param("/use_dials", default = False)
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
 

