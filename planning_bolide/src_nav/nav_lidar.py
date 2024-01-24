#!/usr/bin/env python3

__author__ = "Loris OUMBICHE"
__status__ = "Development"
__version__ = "3.0.0"


#%% IMPORTS
import numpy as np
import rospy
import sys
from control_bolide.msg import SpeedDirection
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


#%% FUNCTIONS

def nav_3_dials(dist_left:float, dist_center:float, dist_right:float, Kspeed:float, Kdir:float) -> tuple:
    """Return the speed and direction of the robot based on 3 dials range data."""
    # Compute the speed command
    speed_cmd = Kspeed * dist_center

    # Limit the speed command
    if speed_cmd > 1:
        speed_cmd = 1
    elif speed_cmd < 0.2:
        speed_cmd = 0.2

    # Compute the direction command
    dir_cmd = - Kdir * (dist_right - dist_left) / speed_cmd

    # Limit the direction command
    if dir_cmd > 1:
        dir_cmd = 1
    elif dir_cmd < -1:
        dir_cmd = -1

    return speed_cmd, dir_cmd




#%% CLASS
class NavLidar():
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel",SpeedDirection,queue_size=1)
        self.cmd = SpeedDirection()


    def Nav(self, Lidar_data, methode = 1):

        i1 = len(Lidar_data)//3

        i2 = 2 * len(Lidar_data)//3

        methode = 3

        rospy.Subscriber("/param_change_alert", Bool, self.get_gain)

        self.get_gain()

        if methode == 1 : 
            gauche = np.average(Lidar_data[0:70])
            centre = np.average(Lidar_data[80:100])
            droite = np.average(Lidar_data[110:180])

            cmd_vel,cmd_dir = nav_3_dials(gauche, centre, droite, self.Kv, self.Kd)

        if methode == 2 :

            cmd_dir = 0
            centre = np.average(Lidar_data[85:95])
            gauche = np.median(Lidar_data[50:90])
            droite = np.median(Lidar_data[90:130])

            #cmd_dir = - max((200 - gauche)/200, 0) + max((200 - droite)/200,0)
            arg = ((np.argmax(Lidar_data) - 90)/90) **2
            cmd_dir = max(-1,min(arg, 1)) * self.Kd
            cmd_vel = min((centre**0.5 /2), 1) * self.Kv

        if methode == 3 : 
            gauche = np.average(Lidar_data[0:70])
            centre = np.average(Lidar_data[80:100])
            droite = np.average(Lidar_data[110:180])

            arg = (np.argmax(Lidar_data) - 90)/90
            Ka = 0.2
            
            print("gauche : ",gauche, "centre : ",centre, "droite : ",droite)



            cmd_vel = self.Kv*centre

            if cmd_vel>1: cmd_vel = 1
            if ( 0 < cmd_vel < 0.2): cmd_vel = 0.2
            if cmd_vel<-1:cmd_vel = -1
            
            cmd_dir = - self.Kd*(droite-gauche) / cmd_vel + arg * Ka
            
            if cmd_dir>1: cmd_dir = 1
            if cmd_dir<-1:cmd_dir = -1


        return cmd_vel,cmd_dir
    
    def get_scan(self, msg:LaserScan):
        Lidar_data = msg.ranges
        self.cmd.speed,self.cmd.direction = self.Nav(Lidar_data)
        self.pub.publish(self.cmd)


    def get_gain(self, value = True):
        self.Kd = rospy.get_param("/gain_direction", default = 0.8)
        self.Kv = rospy.get_param("/gain_vitesse", default = 0.33)


def listener(s:NavLidar):
    rospy.Subscriber('lidar_data',LaserScan,s.get_scan)
    rospy.spin()


if __name__ == '__main__' :
    rospy.init_node('Nav_LIDAR')
    s = NavLidar()

    try : 
        listener(s)
    except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit()
 

