#!/usr/bin/env python3

import numpy as np
import rospy
import sys
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class odom_imu:
    def __init__(self):
        self.pub = rospy.Publisher('Odom',Odometry,queue_size=10)
        self.Odom = Odometry()
        self.Odom_broadcaster = tf.TransformBroadcaster()
        self.orientation = []
        self.angular_velocity = []
        self.linear_acceleration = []

        self.current_time = rospy.Time.now()
        self.last_time =rospy.Time.now()

        self.x_pos = 0
        self.y_pos = 0
        self.theta_pos = 0
        self.dx = 0
        self.dy = 0
        self.dtheta =0

    def integration(self):     # From back wheel location
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()

        # Angular update
        self.dtheta = self.angular_velocity.z
        self.theta_pos = self.orientation.z

        # Linear update
        self.dx += dt*(self.linear_acceleration.x*np.cos(self.theta_pos) - self.linear_acceleration.y*np.sin(self.theta_pos))
        self.dy += dt*(self.linear_acceleration.x*np.sin(self.theta_pos) + self.linear_acceleration.y*np.cos(self.theta_pos))

        self.x_pos += dt*self.dx      
        self.y_pos += dt*self.dy

        self.last_time = self.current_time

    def update(self):
        # Odom position
        Odom_quat = tf.transformations.quaternion_from_euler(0,0,self.theta_pos)
        self.Odom.pose.pose = Pose(Point(self.x_pos,self.y_pos,0.0),Quaternion(*Odom_quat))

        # Odom speed
        self.Odom.twist.twist = Twist(Vector3(self.dx,self.dy,0.0),Vector3(0.0,0.0,self.dtheta))

        # Transform
        self.Odom_broadcaster.sendTransform((self.x_pos,self.y_pos,0.0),Odom_quat,self.current_time,"base_link","odom")
        
        self.Odom.header.stamp = self.current_time
        self.Odom.header.frame_id = "odom"
        self.Odom.child_frame_id = "base_link"

        # Publish Topic
        self.pub.publish(self.Odom)
        
    def get_imu(self,msg:Imu):
        self.orientation = msg.orientation
        self.angular_velocity = msg.angular_velocity
        self.linear_acceleration = msg.linear_acceleration
        s.integration()
        s.update()

def listener(s:odom_imu):
    rospy.Subscriber('raw_imu_data',Imu,s.get_imu)
    rospy.spin()   

if __name__ == '__main__' :
    rospy.init_node('imu_odom_process')
    s = odom_imu()
    try : 
        listener(s)
    except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit()