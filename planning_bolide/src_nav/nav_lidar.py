#!/usr/bin/env python3

import numpy as np
import rospy
import sys
from control_bolide.msg import SpeedDirection
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener

class Nav_LIDAR():
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel",SpeedDirection,queue_size=1)
        self.cmd = SpeedDirection()
        self.space_pressed = False
        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        if key == Key.space:
            self.space_pressed = True

    def on_release(self, key):
        if key == Key.space:
            self.space_pressed = False

    def Nav(self, Lidar_data, methode = 1):

        i1 = len(Lidar_data)//3

        i2 = 2 * len(Lidar_data)//3

        methode = 1

        rospy.Subscriber("/param_change_alert", Bool, self.get_gain)

        self.get_gain()

        if methode == 1 : 
            gauche = np.average(Lidar_data[0:70])
            centre = np.average(Lidar_data[80:100])
            droite = np.average(Lidar_data[110:180])

            print("gauche : ",gauche, "centre : ",centre, "droite : ",droite)



            cmd_vel = self.Kv*centre

            if cmd_vel>1: cmd_vel = 1
            if ( 0 < cmd_vel < 0.2): cmd_vel = 0.2
            if cmd_vel<-1:cmd_vel = -1
            if (self.space_pressed == True): cmd_vel = 2
            cmd_dir = - self.Kd*(droite-gauche) / cmd_vel
            if cmd_dir>1: cmd_dir = 1
            if cmd_dir<-1:cmd_dir = -1

        if methode == 2 :

            cmd_dir = 0
            centre = np.average(Lidar_data[85:95])
            gauche = np.median(Lidar_data[50:90])
            droite = np.median(Lidar_data[90:130])

            #cmd_dir = - max((200 - gauche)/200, 0) + max((200 - droite)/200,0)
            arg = ((np.argmax(Lidar_data) - 90)/90) **2
            cmd_dir = max(-1,min(arg, 1)) * self.Kd
            cmd_vel = min((centre**0.5 /2), 1) * self.Kv

        return cmd_vel,cmd_dir
    
    def get_scan(self, msg:LaserScan):
        Lidar_data = msg.ranges
        self.cmd.speed,self.cmd.direction = self.Nav(Lidar_data)
        self.pub.publish(self.cmd)

    def get_gain(self, value = True):
        self.Kd = rospy.get_param("/gain_direction", default = 0.8)
        self.Kv = rospy.get_param("/gain_vitesse", default = 0.33)


def listener(s:Nav_LIDAR):
    rospy.Subscriber('lidar_data',LaserScan,s.get_scan)
    rospy.spin()


if __name__ == '__main__' :
    rospy.init_node('Nav_LIDAR')
    s = Nav_LIDAR()

    try : 
        listener(s)

    except (rospy.ROSInterruptException, KeyboardInterrupt) : 
        s.shutdown()
        sys.exit()
 
