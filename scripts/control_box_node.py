#! /usr/bin/env python3

from dataclasses import dataclass
from typing import List

import rospy
import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
from std_msgs.msg import UInt8, UInt8MultiArray
from control_box.msg import Button

class RotarySwitch:
    def __init__(self, pins: List[int], name: str):
        self.pins = pins
        for pin in pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.name = name
        self.pub = rospy.Publisher(f"/controller_box/{self.name}", UInt8MultiArray, queue_size=10)

    def spin(self):
        states = []
        for pin in self.pins:
            states.append(GPIO.input(pin) == GPIO.HIGH)

        msg = UInt8MultiArray()
        msg.data = states
        self.pub.publish(msg)

class Switch:
    def __init__(self, pin: int, name: str, no: bool=True):
        self.pin = pin
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.name = name
        self.pub = rospy.Publisher(f"/controller_box/{self.name}", Button, queue_size=10)
        self.no = no

    def spin(self):

        msg = Button()
        msg.header.stamp = rospy.Time.now()
        msg.pin = self.pin
        msg.name = self.name

        state = GPIO.input(self.pin)
        if state == GPIO.HIGH:
            msg.state = (1 if self.no else 0)
        else:
            msg.state = (0 if self.no else 1)

        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("controller_box")
    rate = rospy.Rate(30)

    GPIO.setwarnings(False) # Ignore warning for now
    GPIO.setmode(GPIO.BCM) # Use physical pin numbering

    inputs = [
        Switch(26, "estop", False),
        Switch(19, "switch", False),
        RotarySwitch([21, 20, 16, 18, 25, 24], "cmd_vel_muxer")
    ]


    while not rospy.is_shutdown():
        for i in inputs:
            i.spin()

        rate.sleep()

