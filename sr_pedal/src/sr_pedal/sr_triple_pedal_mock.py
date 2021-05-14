#!/usr/bin/python
# -*- coding: latin-1 -*-

# Copyright 2020 Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import rospy
import sys
import select
from pynput import keyboard
from sr_pedal.msg import Status
from threading import Lock, Thread


class SrPedalMock():
    def __init__(self, connected=True, left_pressed=False, middle_pressed=False, right_pressed=False, rate=20,
                 keyboard_control=False):
        self._lock = Lock()
        self._rate = rospy.Rate(rate)
        self._publisher = rospy.Publisher('/sr_pedal/status', Status, queue_size=20)
        self._status = Status()
        self._stopping = False
        self._thread = Thread(target=self._publish_thread)
        self._status.connected = connected
        self._status.left_pressed = left_pressed
        self._status.middle_pressed = middle_pressed
        self._status.right_pressed = right_pressed
        self._ctrl_pressed = False
        self._shift_pressed = False
        self._alt_pressed = False
        self._kb_listener = None
        if keyboard_control:
            self._kb_listener = keyboard.Listener(on_press=self._on_keyboard_press,
                                                  on_release=self._on_keyboard_release)

    def run(self):
        if self._kb_listener is not None:
            self._kb_listener.start()
            rospy.loginfo("Mock pedal keyboard control enabled.\n" +
                          "╔============╦=========================╗\n" +
                          "║  Control   ║         Effect          ║\n" +
                          "╠============╬=========================╣\n" +
                          "║ ctrl+alt+5 ║ Toggle pedal connection ║\n" +
                          "║ ctrl+alt+6 ║ Toggle left pedal       ║\n" +
                          "║ ctrl+alt+7 ║ Toggle middle pedal     ║\n" +
                          "║ ctrl+alt+8 ║ Toggle right pedal      ║\n" +
                          "╚============╩=========================╝")
        self._thread.start()

    def _publish_thread(self):
        rospy.loginfo("Starting mock pedal publishing.")
        while not (self._stopping or rospy.is_shutdown()):
            self._lock.acquire()
            self._status.header.stamp = rospy.Time.now()
            self._publisher.publish(self._status)
            self._lock.release()
            self._rate.sleep()

    def stop(self):
        rospy.loginfo("Stopping mock pedal publishing.")
        self._stopping = True
        if self._kb_listener is not None:
            self._kb_listener.stop()
        self._thread.join()

    def set_status(self, connected=None, left_pressed=None, middle_pressed=None, right_pressed=None):
        self._lock.acquire()
        if connected is not None:
            self._status.connected = connected
            if connected:
                rospy.loginfo("Mock pedal now connected.")
            else:
                rospy.loginfo("Mock pedal now disconnected.")
        if left_pressed is not None:
            self._status.left_pressed = left_pressed
            if left_pressed:
                rospy.loginfo("Mock left pedal now pressed.")
            else:
                rospy.loginfo("Mock left pedal now released.")
        if middle_pressed is not None:
            self._status.middle_pressed = middle_pressed
            if middle_pressed:
                rospy.loginfo("Mock middle pedal now pressed.")
            else:
                rospy.loginfo("Mock middle pedal now released.")
        if right_pressed is not None:
            self._status.right_pressed = right_pressed
            if right_pressed:
                rospy.loginfo("Mock right pedal now pressed.")
            else:
                rospy.loginfo("Mock right pedal now released.")
        self._lock.release()

    def _on_keyboard_press(self, key):
        if key == keyboard.Key.ctrl:
            self._ctrl_pressed = True
        elif key == keyboard.Key.ctrl:
            self._ctrl_pressed = True
        elif key == keyboard.Key.shift:
            self._shift_pressed = True
        elif key == keyboard.Key.alt:
            self._alt_pressed = True
        elif self._ctrl_pressed and self._alt_pressed and not self._shift_pressed:
            if str(key) == "u'5'":
                self.set_status(connected=not self._status.connected)
            elif str(key) == "u'6'":
                self.set_status(left_pressed=not self._status.left_pressed)
            elif str(key) == "u'7'":
                self.set_status(middle_pressed=not self._status.middle_pressed)
            elif str(key) == "u'8'":
                self.set_status(right_pressed=not self._status.right_pressed)

    def _on_keyboard_release(self, key):
        if key == keyboard.Key.ctrl:
            self._ctrl_pressed = False
        elif key == keyboard.Key.shift:
            self._shift_pressed = False
        elif key == keyboard.Key.alt:
            self._alt_pressed = False


if __name__ == "__main__":
    rospy.init_node("sr_pedal_mock")
    rate = rospy.get_param("~rate", 20)
    connected = rospy.get_param("~connected", True)
    left_pressed = rospy.get_param("~left_pressed", False)
    middle_pressed = rospy.get_param("~middle_pressed", False)
    right_pressed = rospy.get_param("~right_pressed", False)
    keyboard_control = rospy.get_param("~keyboard_control", False)
    sr_pedal_mock = SrPedalMock(connected, left_pressed, middle_pressed, right_pressed, rate, keyboard_control)
    sr_pedal_mock.run()
    rospy.spin()
    sr_pedal_mock.stop()
