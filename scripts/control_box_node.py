#! /usr/bin/env python3

from dataclasses import dataclass
from typing import List, Union

import rospy
from RPi import GPIO
from control_box.msg import Button as ButtonMsg
from control_box.msg import RotarySwitch as RotarySwitchMsg


class RotarySwitch:
    def __init__(self, pins: List[int], name: str):
        self.pins = pins
        self.name = name
        self.pub = rospy.Publisher(f"/controller_box/{self.name}", RotarySwitchMsg, queue_size=10)

        for pin in pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def spin(self):
        msg = RotarySwitchMsg()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.name
        msg.pins = self.pins
        msg.states = [GPIO.input(pin) == GPIO.HIGH for pin in self.pins]

        self.pub.publish(msg)

class Button:
    def __init__(self, pin: int, name: str, no: bool=True):
        self.pin = pin
        self.name = name
        self.pub = rospy.Publisher(f"/controller_box/{self.name}", ButtonMsg, queue_size=10)
        self.no = no

        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def spin(self):
        msg = ButtonMsg()
        msg.header.stamp = rospy.Time.now()
        msg.pin = self.pin
        msg.name = self.name
        msg.state = GPIO.input(self.pin) if self.no else not GPIO.input(self.pin)

        self.pub.publish(msg)

def get_devices_from_params() -> List[Union[RotarySwitch, Button]]:
    # Get all parameters within the namespace
    params = rospy.get_param("control_box")

    # Iterate through parameters and add the devices
    devices = []
    for key, data in params.items():
        # Check for the fields that all devies must have
        # TODO: There must be a smarter way to do this.
        # Maybe a helper function and standardized message?
        if "name" not in data:
            print(f"device: {key} does not contain a name... skipping...")
            continue
        if "type" not in data:
            print(f"device: {key} does not contain a name... skipping...")
            continue

        name = data["name"]
        type_ = data["type"]

        # "Switch" based on the type. For each type check that the required fields are
        # present. Because everything is YAML and in the parameter server no typecasting
        # is required
        if data["type"] == "button":
            if "pin" not in data:
                print("device: {key} does not contain a pin... skipping...")
                continue
            if "normally_open" not in data:
                print("device: {key} does not contain a normally open field... skipping...")
                continue

            devices.append(
                Button(
                    data["pin"],
                    data["name"],
                    data["normally_open"]
                )
            )

        if data["type"] == "rotary_switch":
            if "pins" not in data:
                print("device: {key} does not contain a pins field... skipping...")
                continue

            devices.append(
                RotarySwitch(
                    data["pins"],
                    data["name"]
                )
            )

    # Return the devices list
    return devices

if __name__ == "__main__":
    # Set up the ROS node
    rospy.init_node("controller_box")
    rate = rospy.Rate(30)

    # Set up the GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # Gather all devices from parameter server
    inputs = get_devices_from_params()

    # Exit early if there are no devices
    if len(inputs) == 0:
        print("No devices defined... Exiting...")
        exit(1)

    # Loop, query, publish
    while not rospy.is_shutdown():
        for i in inputs:
            i.spin()
        rate.sleep()

