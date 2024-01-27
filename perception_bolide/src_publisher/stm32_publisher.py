#!/usr/bin/env python3

__author__ = "Quentin Rolland"
__email__ = "quentin.rolland@ensea.fr"
__status__ = "Development"
__version__ = "1.0.0"

import rospy
import spidev 
from std_msgs.msg import Float32MultiArray


class STM32DataReceiver:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('stm32_publisher')

        # variables
        self.sensor_data = Float32MultiArray() # table for the sensors data

        self.bus=0 
        self.device=1

        self.spi = spidev.SpiDev()
        self.spi.open(self.bus,self.device)
        self.spi.mode = 0
        self.spi.max_speed_hz = 500000  # Définir la vitesse maximale du bus SPI à 500 kHz

        self.pub = rospy.Publisher('stm32_sensors', Float32MultiArray, queue_size=10)
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.3), self.receiveSensorData)
        rospy.spin()

    def receiveSensorData(self, _):
        data = self.spi.xfer2([0]*16)  # Envoyer 16 bytes de données nulles et recevoir 16 bytes de données en retour

        #roll = (data[1] << 8) | data[0]  # Concatener les octets de poids fort et faible pour reconstituer la valeur
        yaw = (data[1] << 8) | data[0]
        #pitch = (data[5] << 8) | data[4]
        acceleration_x = (data[3] << 8) | data[2]
        speed_mm_s = (data[5] << 8) | data[4]
        distance_US = (data[7] << 8) | data[6]
        IR_Left = (data[9] << 8) | data[8]
        IR_Right = (data[11] << 8) | data[10]

        self.sensor_data.data = [yaw, acceleration_x, speed_mm_s, distance_US, IR_Left, IR_Right]
        self.pub.publish(self.sensor_data)
        
if __name__ == '__main__':
    try:
        # Create a STM32DataReceiver and start it
        STM32_data = STM32DataReceiver()
    except rospy.ROSInterruptException:
        # If a ROSInterruptException occurs, exit the program
        exit(0)
     
