#!/usr/bin/env python3

__author__ = "Quentin Rolland"
__email__ = "quentin.rolland@ensea.fr"
__status__ = "Tested"
__version__ = "1.0.0"

import rospy
import spidev 
from std_msgs.msg import Float32MultiArray




# Definitely not the most efficient way of doing this, but we're transmitting 16 bytes so it's OK.
# Ideally we'd do all this in Cpp, but I don't think there is a spidev equivalent in Cpp, and it's 
# not like we're being limited by this script. 
def crc32mpeg2(buf, crc=0xffffffff):
    for val in buf:
        crc ^= val << 24
        for _ in range(8):
            crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
    return crc

    

def receiveSensorData():
    data = spi.xfer2([0x45]*16)  # Envoyer 16 bytes de données nulles et recevoir 16 bytes de données en retour

    vbat = 0
    yaw = 0
    ir_gauche = 0
    ir_droit = 0
    speed = 0
    distance_US = 0


    if (not crc32mpeg2(data)):
        vbat = (data[0] << 8) | data[1]
        yaw = (data[2] << 8) | data[3]
        ir_gauche = (data[4] << 8) | data[5]
        ir_droit = (data[6] << 8) | data[7]
        speed = (data[8] << 8) | data[9]
        distance_US = (data[10] << 8) | data[11]
    

    sensor_data.data = [yaw, speed, ir_gauche, ir_droit, distance_US]
    pub.publish(sensor_data)
        
if __name__ == '__main__':
    # variables
    sensor_data = Float32MultiArray() # table for the sensors data

    bus=0 
    device=1

    spi = spidev.SpiDev()
    spi.open(bus,device)
    spi.mode = 0
    spi.max_speed_hz = 100000  # Définir la vitesse maximale du bus SPI à 100 kHz

    # Create a STM32DataReceiver and start it
    rospy.init_node('stm32_publisher')
    pub = rospy.Publisher('stm32_sensors', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(150)
    while(not rospy.is_shutdown()):
        receiveSensorData()
        rate.sleep()