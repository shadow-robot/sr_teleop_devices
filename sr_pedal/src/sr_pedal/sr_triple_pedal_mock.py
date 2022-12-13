#!/usr/bin/python3

# -*- coding: latin-1 -*-

# Copyright 2020-2022 Shadow Robot Company Ltd.
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

import sys
from threading import Lock, Thread
import rospy
from sr_pedal.msg import Status

# If we're in a non-X-server (e.g. CI) environment, this import will fail
try:
    from pynput import keyboard
except ImportError as err:
    print("SrPedalMock could not import pynput module; likely because there is no available X server. "
          "Pedal mock keyboard control disabled.")


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
        self._alt_pressed = False
        self._kb_listener = None
        # If pynput failed to import, disable keyboard control
        keyboard_control = keyboard_control and "pynput" in sys.modules
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
                          "║ ctrl+alt+9 ║ Toggle left and right   ║\n" +
                          "╚============╩=========================╝")
        self._thread.start()

    def _publish_thread(self):
        rospy.loginfo("Starting mock pedal publishing.")
        while not (self._stopping or rospy.is_shutdown()):
            with self._lock:
                self._status.header.stamp = rospy.Time.now()
                self._publisher.publish(self._status)
            self._rate.sleep()

    def stop(self):
        rospy.loginfo("Stopping mock pedal publishing.")
        self._stopping = True
        if self._kb_listener is not None:
            self._kb_listener.stop()
        self._thread.join()

    def set_status(self, connected=None, left_pressed=None, middle_pressed=None, right_pressed=None):
        with self._lock:
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

    def toggle_status(self, status):
        if status == "connected":
            self._status.connected = not self._status.connected
        elif status == "left":
            self._status.left_pressed = not self._status.left_pressed
        elif status == "middle":
            self._status.middle_pressed = not self._status.middle_pressed
        elif status == "right":
            self._status.right_pressed = not self._status.right_pressed
        else:
            rospy.logwarn(f'Cannot toggle status of unknown pedal state "{status}".')

    def _on_keyboard_press(self, key):
        if key == keyboard.Key.ctrl:
            self._ctrl_pressed = True
        elif key == keyboard.Key.alt:
            self._alt_pressed = True

        if self._ctrl_pressed and self._alt_pressed:
            if str(key) == "'5'":
                self.set_status(connected=not self._status.connected)
            elif str(key) == "'6'":
                self.set_status(left_pressed=not self._status.left_pressed)
            elif str(key) == "'7'":
                self.set_status(middle_pressed=not self._status.middle_pressed)
            elif str(key) == "'8'":
                self.set_status(right_pressed=not self._status.right_pressed)
            elif str(key) == "'9'":
                self.set_status(left_pressed=not self._status.left_pressed)
                self.set_status(right_pressed=not self._status.right_pressed)

    def _on_keyboard_release(self, key):
        if key == keyboard.Key.ctrl:
            self._ctrl_pressed = False
        elif key == keyboard.Key.alt:
            self._alt_pressed = False


if __name__ == "__main__":
    rospy.init_node("sr_pedal_mock")
    publishing_rate = rospy.get_param("~rate", 20)
    is_connected = rospy.get_param("~connected", True)
    is_left_pressed = rospy.get_param("~left_pressed", False)
    is_middle_pressed = rospy.get_param("~middle_pressed", False)
    is_right_pressed = rospy.get_param("~right_pressed", False)
    is_keyboard_control = rospy.get_param("~keyboard_control", False)
    sr_pedal_mock = SrPedalMock(is_connected, is_left_pressed,
                                is_middle_pressed, is_right_pressed,
                                publishing_rate, is_keyboard_control)
    sr_pedal_mock.run()
    rospy.spin()
    sr_pedal_mock.stop()
