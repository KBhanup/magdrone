#!/usr/bin/python

import rospy as rp

from sensor_msgs.msg import Joy

class joyListenerNode():

    def __init__(self):
        rp.init_node("my_joy_listener"")

        # Set up Subscriber
        self.joy_sub = rp.Subscriber("/joy_node", Joy, self.joy_callback, queue_size=1)

        rp.spin()

    def joy_callback(self, data):
        print(data.axes[0])
        print(data.buttons[0])

joy_listener = joyListenerNode()