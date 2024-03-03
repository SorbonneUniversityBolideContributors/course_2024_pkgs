#!/usr/bin/env python3

__author__ = "Eliot CHRISTON"
__status__ = "Tested"

#%% IMPORTS
import rospy
from std_msgs.msg import Float32
from control_bolide.msg import SpeedDirection
from pynput.keyboard import Key, Listener


#%% CLASS
class KeyboardController:
    def __init__(self):

        rospy.init_node('teleop_node')
        rospy.loginfo("Teleop node started, Keyboard interupt (ctrl+c will stop the node")

        # init publisher
        self.pub = rospy.Publisher('cmd_vel', SpeedDirection, queue_size=10)

        # init keyboard listener
        self.listener = Listener(on_press=self.on_key_press, on_release=self.on_key_release)

        # init speed and direction
        self.current_speed = 0.0
        self.current_direction = 0.0

        # init timer for publishing
        # The timer callback is called every 0.4 seconds so that the robot keeps getting commands
        self.timer = rospy.Timer(rospy.Duration(0.4), self.timer_callback)

        # controls
        self.key_mapping = {
            Key.up: 'UP',
            Key.down: 'DOWN',
            Key.left: 'LEFT',
            Key.right: 'RIGHT',
            Key.space: 'BRAKE',
            'z': 'UP',
            'q': 'LEFT',
            's': 'DOWN',
            'd': 'RIGHT',
        }

        # print controls
        key_mapping_str = '\n'.join([f'\t{key}: {value}' for key, value in self.key_mapping.items()])
        print(f"Key control mapping:\n{key_mapping_str}\n")

    def timer_callback(self, event):
        self.publish_speed_direction()

    def publish_speed_direction(self):
        self.pub.publish(SpeedDirection(speed=self.current_speed, direction=self.current_direction))

    def on_key_press(self, in_key):
        key = self.get_key_char(in_key)
        if key in self.key_mapping.keys():
            self.perform_action(self.key_mapping[key])
        else:
            rospy.logwarn(f"No action for key: {key}")

    def on_key_release(self, in_key):
        key = self.get_key_char(in_key)
        if key in self.key_mapping.keys():
            self.perform_action(self.key_mapping[key], coeff=0)

    def get_key_char(self, key):
        """Returns the character representation of a key press event"""
        if hasattr(key, 'char'):
            return key.char
        else:
            return key

    def perform_action(self, action, coeff=1.0):
        """Performs the action on the robot
        the coeff is used to reverse the action if needed, or to stop the robot"""
        if action == 'UP':
            self.current_speed = 1.0 * coeff
        elif action == 'DOWN':
            self.current_speed = -1.0 * coeff
        elif action == 'LEFT':
            self.current_direction = -1.0 * coeff
        elif action == 'RIGHT':
            self.current_direction = 1.0 * coeff
        elif action == 'BRAKE':
            self.current_speed = 2.0 * coeff
        else:
            rospy.logwarn(f"Unknown action: {action}")
        self.publish_speed_direction()


#%% MAIN
if __name__ == '__main__':
    controller = KeyboardController()

    # Set up keyboard events dynamically
    try:
        with controller.listener:
            rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt received. Shutting down...")
    finally:
        # Clean up and unregister keyboard events
        controller.timer.shutdown()
        controller.listener.stop()
        controller.listener.join()