#!/usr/bin/python

# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import evdev
from sr_pedal.msg import Status

class SrPedal():
    def __init__(self):
        self.device = None
        self.product_name = 'VEC  VEC USB Footpedal'
        self.left_pressed = False
        self.middle_pressed = False
        self.right_pressed = False
        self.message = Status()
        self.publisher = rospy.Publisher('sr_pedal/status', Status, queue_size=10)
        self.goal_rate = rospy.Rate(20)

    def run(self):
        while not rospy.is_shutdown():
            self.connect()
            while not rospy.is_shutdown():
                try:
                    keys = self.device.active_keys()
                    self.left_pressed = 256 in keys
                    self.middle_pressed = 257 in keys
                    self.right_pressed = 258 in keys
                    self.publish()
                    self.goal_rate.sleep()
                except IOError:
                    self.disconnect()
                    break

    def connect(self):
        rospy.loginfo("Waiting for pedal...")
        while ((not rospy.is_shutdown()) and self.device is None):
            self.device = self.get_pedal()
            self.publish()
            if self.device is not None:
                rospy.loginfo("Pedal connected.")
                break
            else:
                self.goal_rate.sleep()

    def get_pedal(self):
        pedal = None
        device_paths = evdev.list_devices()
        for device_path in device_paths:
            try:
                device = evdev.InputDevice(device_path)
                if device.name == self.product_name:
                    pedal = device
                    break
            except OSError:
                pass # Probably because the device doesn't exist any more
        return pedal

    def disconnect(self):
        rospy.logwarn("Pedal disconnected.")
        # Try to both ungrab and close the device. Necessary for opening the device again when it is reconnected.
        try:
            self.device.ungrab()
        except:
            pass
        try:
            self.device.close()
        except:
            pass
        self.device = None
        self.left_pressed = False
        self.middle_pressed = False
        self.right_pressed = False
        self.publish()

    def publish(self, time = None):
        if time is None:
            time = rospy.Time.now()
        self.message.header.stamp = time
        self.message.connected = self.device is not None
        self.message.left_pressed = self.left_pressed
        self.message.middle_pressed = self.middle_pressed
        self.message.right_pressed = self.right_pressed
        self.publisher.publish(self.message)


if __name__ == "__main__":
    rospy.init_node("sr_pedal")
    rate = rospy.get_param("~rate", 20)
    print(rate)
    sr_pedal = SrPedal()
    sr_pedal.goal_rate = rospy.Rate(rate)
    sr_pedal.run()
