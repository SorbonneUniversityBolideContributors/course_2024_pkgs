#!/usr/bin/env python3

__author__ = "Nicolas Hammje"
__email__ = "me@nicolashammje.fr"
__status__ = "Tested"
__version__ = "1.0.0"

import rospy
import spidev 
from std_msgs.msg import Float32MultiArray




class STM32_Parser:
    def __init__(self):
        self.BAUDRATE = 100000
        
        bus=0 
        device=1

        self.sensor_pub = rospy.Publisher('stm32_sensors', Float32MultiArray, queue_size=10)
        self.spi = spidev.SpiDev()
        self.spi.open(bus,device)
        self.spi.mode = 0
        self.spi.max_speed_hz = self.BAUDRATE 


        self.vbat = 0
        self.yaw = 0
        self.ir_gauche = 0
        self.ir_droit = 0
        self.speed = 0
        self.distance_US = 0

        self.sensor_data = Float32MultiArray() # table for the sensors data



    # Definitely not the most efficient way of doing this, but we're transmitting 16 bytes so it's OK.
    # Ideally we'd do all this in Cpp, but I don't think there is a spidev equivalent in Cpp, and it's 
    # not like we're being limited by this script. 
    def crc32mpeg2(self,buf, crc=0xffffffff):
        for val in buf:
            crc ^= val << 24
            for _ in range(8):
                crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
        return crc

        

    def receiveSensorData(self):
        data = self.spi.xfer2([0x45]*16)  # SPI happens simultaneously, so we need to send to receive. 


        if (not self.crc32mpeg2(data)):
            self.vbat = (data[0] << 8) | data[1]
            self.yaw = (data[2] << 8) | data[3]
            self.ir_gauche = (data[4] << 8) | data[5]
            self.ir_droit = (data[6] << 8) | data[7]
            self.speed = (data[8] << 8) | data[9]
            self.distance_US = (data[10] << 8) | data[11]

        print(self.vbat)

        self.sensor_data.data = [self.yaw, self.speed, self.ir_gauche, self.ir_droit, self.distance_US]
        self.sensor_pub.publish(self.sensor_data)
        
if __name__ == '__main__':

    Parser = STM32_Parser()    
    rospy.init_node('stm32_publisher')
    rate = rospy.Rate(150)
    while(not rospy.is_shutdown()):
        Parser.receiveSensorData()
        rate.sleep()
