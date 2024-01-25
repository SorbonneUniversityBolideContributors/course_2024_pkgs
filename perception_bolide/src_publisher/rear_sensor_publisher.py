#!/usr/bin/env python3

__author__ = "Quentin Rolland"
__email__ = "quentin.rolland@ensea.fr"
__status__ = "Development"
__version__ = "1.0.0"

import rospy
from sensor_msgs.msg import Range
from perception_bolide.msg import MultipleRange
from std_msgs.msg import Float32MultiArray

class RearSensors:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('rear_sensor_publisher')

        # SUBSCRIBER ========================================
        rospy.Subscriber("STM32_sensors_topic", Float32MultiArray, self.callback)
        # PUBLISHER =========================================
        self.pub = rospy.Publisher('raw_rear_range_data', MultipleRange, queue_size=10)

        # defining ranges of the IR
        self.ir_min_range = 0.06
        self.ir_max_range = 0.3

        # defining ranges of the US
        self.sonar_min_range = 0.07
        self.sonar_max_range = 0.5

        # init the static data of the sensors
        self.sensors_init()

        rospy.spin() # wait for the callback to be called

    def sensors_init(self):
        """Initialize the sensors settings and the MultiRange message"""
        self.multiRangeFrame = MultipleRange()

        # Rear IR sensors ===============================================================
        self.multiRangeFrame.IR_rear_left = Range()
        self.multiRangeFrame.IR_rear_left.header.frame_id = "rear_ir_range_frame"
        self.multiRangeFrame.IR_rear_left.radiation_type = Range.INFRARED
        self.multiRangeFrame.IR_rear_left.min_range = self.ir_min_range
        self.multiRangeFrame.IR_rear_left.max_range = self.ir_max_range

        self.multiRangeFrame.IR_rear_right = Range()
        self.multiRangeFrame.IR_rear_right.header.frame_id = "rear_ir_range_frame"
        self.multiRangeFrame.IR_rear_right.radiation_type = Range.INFRARED
        self.multiRangeFrame.IR_rear_right.min_range = self.ir_min_range
        self.multiRangeFrame.IR_rear_right.max_range = self.ir_max_range
        
        # Rear sonar sensor =============================================================
        self.multiRangeFrame.Sonar_rear = Range()
        self.multiRangeFrame.Sonar_rear.header.frame_id = "rear_sonar_range_frame"
        self.multiRangeFrame.Sonar_rear.radiation_type = Range.ULTRASOUND
        self.multiRangeFrame.Sonar_rear.min_range = self.sonar_min_range
        self.multiRangeFrame.Sonar_rear.max_range = self.sonar_max_range

    def callback(self, data:Float32MultiArray) :
        """ Callback function called when a message is received on the subscribed topic"""
        
        # retrieving Rear data (2xIR + sonar) from the STM32_sensors msg
        Sonar_rear = data.data[3] #sonar
        IR_rear_left = data.data[4] #IR left
        IR_rear_right = data.data[5] #IR right

        # Process sonar data
        Sonar_rear = Sonar_rear/100 # passage de cm à m

        # Process IR's data
        # ========== conversion mV --> m ==========
        # Left ====================================
        IR_rear_left = IR_rear_left/1000 # conversion mV --> V
        if(IR_rear_left == 0):
            IR_rear_left = self.ir_max_range*100 #because we convert in m later
        else :
            IR_rear_left = 15.38/IR_rear_left - 0.42 # conversion V --> cm
        # using component datasheet
        # ... and with a bit of experimentation and test to adapt the value
        IR_rear_left = IR_rear_left/100 # conversion cm --> m
        if(IR_rear_left < self.ir_min_range): # on met à 0 en dessous d'un certain seuil, cf doc
            IR_rear_left = 0
        elif(IR_rear_left > self.ir_max_range):
            IR_rear_left = self.ir_max_range

        # Right ===================================
        IR_rear_right = IR_rear_right/1000 # conversion mV --> V
        if(IR_rear_right == 0):
            IR_rear_right = self.ir_max_range*100 #because we convert in m later
        else :
            IR_rear_right = 15.38/IR_rear_right - 0.42 # using component datasheet
        # using component datasheet
        # ... and with a bit of experimentation and test to adapt the value
        IR_rear_right = IR_rear_right/100 # conversion cm --> m
        if(IR_rear_right < self.ir_min_range): # on met à 0 en dessous d'un certain seuil, cf doc
            IR_rear_right = 0
        elif(IR_rear_right > self.ir_max_range):
            IR_rear_right = self.ir_max_range

        self.multiRangeFrame.Sonar_rear.range = Sonar_rear
        self.multiRangeFrame.Sonar_rear.header.stamp = rospy.Time.now()
        self.multiRangeFrame.IR_rear_left.range = IR_rear_left
        self.multiRangeFrame.IR_rear_left.header.stamp = rospy.Time.now()
        self.multiRangeFrame.IR_rear_right.range = IR_rear_right
        self.multiRangeFrame.IR_rear_right.header.stamp = rospy.Time.now()

        self.pub.publish(self.multiRangeFrame)
        

if __name__ == '__main__':
    try:
        # Create a RearSenors and start it
        Rear_data = RearSensors()
    except rospy.ROSInterruptException:
        # If a ROSInterruptException occurs, exit the program
        exit(0)