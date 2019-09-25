#!/usr/bin/python

# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# This file contains an alternate approach to pedal connection, that falls victim to Ubuntu suspending USB devices.
# Try as I might, I couldn't make Ubuntu leave it alone, so switched to the input events approach seen in
# sr_triple_pedal.py. I'll leave this here in case it might be useful later.

import rospy
import usb.core
import usb.util
from sr_pedal.msg import Status


class SrPedal():
    def __init__(self):
        self.device = None
        self.vendor_id = 0x05f3
        self.product_id = 0x00ff
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
                if usb.core.find(idVendor=self.vendor_id, idProduct=self.product_id) is None:
                    self.disconnect()
                    break
                try:
                    value = self.device.read(0x81, 2, self.goal_rate.remaining().to_sec*1000)
                    rospy.logwarn_throttle(1, value)
                    print(value)
                except:
                    pass
                self.publish()
                self.goal_rate.sleep()

    def connect(self):
        rospy.loginfo("Waiting for pedal...")
        while ((not rospy.is_shutdown()) and self.device is None):
            self.device = usb.core.find(idVendor=self.vendor_id, idProduct=self.product_id)
            self.publish()
            if self.device is not None:
                rospy.loginfo("Pedal connected.")
                try:
                    self.device.detach_kernel_driver(0)
                except:
                    pass
                self.device.set_configuration()
                break
            else:
                self.goal_rate.sleep()

    def disconnect(self):
        rospy.logwarn("Pedal disconnected.")
        self.device = None
        self.left_pressed = False
        self.middle_pressed = False
        self.right_pressed = False
        self.publish()

    def publish(self, time=None):
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
    sr_pedal = SrPedal()
    sr_pedal.run()
