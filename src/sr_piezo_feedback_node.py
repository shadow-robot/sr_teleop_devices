#!/usr/bin/env python3

# Copyright 2021 Shadow Robot Company Ltd.
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
from __future__ import division
import argparse
import sys
import math
import rospy
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
from sr_robot_msgs.msg import ShadowPST, BiotacAll
from std_msgs.msg import Float64, Float64MultiArray, Header
import threading
import time
from sr_hand.tactile_receiver import TactileReceiver
import matplotlib.pyplot as plt

from dynamic_reconfigure.server import Server
from sr_piezo_feedback.cfg import SrPiezoFeedbackConfig
from sr_piezo_feedback.msg import PiezoFeedback



class DeviceHandler(threading.Thread):
    def __init__(self, device, fingers, mount):
        super(DeviceHandler, self).__init__()
        self._device_name = device
        self._finger_per_devices = fingers
        self._mount = mount
        self._samplerate = sd.query_devices(self._device_name, 'output')['default_samplerate']

        self._ts = [0] * len(self._finger_per_devices)
        self._oldsignal = [0] * len(self._finger_per_devices)
        self._freq = [1] * len(self._finger_per_devices)
        self._amp = [1] * len(self._finger_per_devices)
        self._signal_range = [-0.1, 0.6]
        self._offset = np.mean(self._signal_range)
        self._amp_factor = np.mean(np.abs(self._signal_range))

        self.signal_publisher = rospy.Publisher('sr_finger_mount/piezo_feedback/output', PiezoFeedback, queue_size=1)
        self.amp_publisher = rospy.Publisher('sr_finger_mount/piezo_feedback/amplitude', PiezoFeedback, queue_size=1)
        self.freq_publisher = rospy.Publisher('sr_finger_mount/piezo_feedback/frequency', PiezoFeedback, queue_size=1)

        self.msg = PiezoFeedback()
        self.msg.header = Header()

    def run(self):
        self.start_piezo(self._finger_per_devices)

    def callback(self, outdata, frames, time, status):
        if status:
            rospy.logwarn(status)

        for i, finger in enumerate(self._finger_per_devices):
            for frame in range(frames):
                outdata[frame, i] = self._offset + self._amp_factor * self._amp[i] * np.sin(self._ts[i])
                self._ts[i] += 2 * np.pi * self._freq[i] / self._samplerate

                if outdata[frame, i] != np.sign(self._oldsignal[i]):
                    self._freq[i] = self._mount._frequencies[finger]
                    self._amp[i] = self._mount._amplitudes[finger]
                self._oldsignal[i] = outdata[frame, i]

                self.msg.header.stamp = rospy.get_rostime()
                self.msg.feedback = Float64(outdata[frame, i])
                self.signal_publisher.publish(self.msg)

    def start_piezo(self, fingers):
        with sd.OutputStream(device=self._device_name, channels=len(fingers), callback=self.callback,
                             samplerate=self._samplerate, blocksize=100, latency='low'):
            while not rospy.is_shutdown():
                rospy.sleep(0.1)


class SrPiezoFeedback():

    CONST_PST_MAX = 1
    CONST_PST_MIN = 0

    CONST_BIOTAC_MAX = 1
    CONST_BIOTAC_MIN = 0

    CONST_FINGERS = ["ff", "mf", "rf", "lf", "th"]
    CONST_ACCEPTABLE_DEVICE_NAMES = ["Boreas DevKit", "BOS1901-KIT"]
    CONST_CHANNELS_PER_DEVICE = 2

    def __init__(self, fingers, hand_id):
        self._used_fingers = fingers
        self._used_devices = []
        self._hand_id = hand_id

        self._contact_time = 0.1
        self._amp_max = 1.0
        self._amp_min = 0.0
        self._freq_min = 1
        self._freq_max = 80

        self._normalized_pressure = 5 * [0]
        self._prev_values = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self._amplitudes = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self._frequencies = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))

        self._used_tactiles = TactileReceiver(self._hand_id).get_tactile_type()
        self._device_handlers = [None, None, None]

        # TO BE REMOVED
        self._start_time = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self.fading_time = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self.fading_amplitudes = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self.fading_frequencies = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))

        if self._used_tactiles == "PST":
            # self._pst_threshold = [395.0, 395.0, 395.0, 395.0, 395.0]
            self._pst_saturation = [550.0, 550.0, 550.0, 550.0, 550.0]
            self._init_thresholds()
            rospy.Subscriber("/"+self._hand_id+"/tactile", ShadowPST, self._pst_tactile_cb)

        elif self._used_tactiles == "biotac":
            try:
                from haptx_tactile_mapping.biotac_sp_minus_mapping import BiotacMapping
                from haptx_msgs.msg import Movables, Movable, Tactor, BiotacAllFloat
                BiotacMapping(self._hand_id)
                rospy.Subscriber("haptx_movables", Movables, self._biotac_tactile_cb)
            except Exception:
                rospy.logerr("Error while importing haptx")

        if not set(self._used_fingers).intersection(set(self.CONST_FINGERS)):
            rospy.logerr("Failed to start node! Used fingers are not allowed: {}".format(self.CONST_FINGERS))
            return

        if not self._check_devices():
            return

        self.initialize()

    def _init_thresholds(self):
        samples_to_collect = 50
        thresholds_to_set = samples_to_collect * [None]
        for i in range(0, samples_to_collect):
            data = rospy.wait_for_message("/"+self._hand_id+"/tactile", ShadowPST)
            thresholds_to_set[i] = [data.pressure]
        self._pst_threshold = np.mean(thresholds_to_set, axis=1)[0]

    def _check_devices(self):
        needed_devices = math.ceil(len(self._used_fingers)/2)
        device_list = sd.query_devices()
        present_devices = 0

        for acceptable_device in self.CONST_ACCEPTABLE_DEVICE_NAMES:
            for device in device_list:
                if acceptable_device in device['name']:
                    self._used_devices.append(device['name'])
                    present_devices += 1

        if needed_devices > present_devices:
            rospy.logerr("Not enough dev kits ({}/{}) connected to handle {} fingers".format(present_devices,
                                                                                             needed_devices,
                                                                                             len(self._used_fingers)))
            return False
        return True

    def initialize(self):
        finger_sets = list(self._sublists())
        for i, finger_set in enumerate(finger_sets):
            device_name = self._used_devices[i]
            time.sleep(1)
            self._device_handlers[i] = DeviceHandler(device_name, finger_set, self)
            self._device_handlers[i].start()

    def _sublists(self):
        for i in range(0, len(self._used_fingers), self.CONST_CHANNELS_PER_DEVICE):
            yield self._used_fingers[i:i + self.CONST_CHANNELS_PER_DEVICE]

    def _process_tactile_data(self, mapped_values):

        for finger in self._used_fingers:
            self.fading_time[finger] = rospy.get_time() - self._start_time[finger]

            if self._prev_values[finger] < 0.03 and np.abs(mapped_values[finger]-self._prev_values[finger]) > 0.02:
                self._start_time[finger] = rospy.get_time()
                rospy.logwarn("zeroing startime for {}".format(finger))

            if self.fading_time[finger] <= self._contact_time:
                a = -4 / (self._contact_time * self._contact_time)
                b = -a * self._contact_time
                fading_factor = a * self.fading_time[finger] * self.fading_time[finger] + b * self.fading_time[finger]
                self.fading_amplitudes[finger] = self._amp_max * 1.0 * fading_factor
                self.fading_frequencies[finger] = self._freq_max * fading_factor
            else:
                self.fading_amplitudes[finger] = 0
                self.fading_frequencies[finger] = 0

            self._amplitudes[finger] = ((mapped_values[finger] - self.CONST_PST_MIN) /
                                        (self.CONST_PST_MAX - self.CONST_PST_MIN)) * \
                (self._amp_max - self._amp_min) + self._amp_min
            self._frequencies[finger] = ((mapped_values[finger] - self.CONST_PST_MIN) /
                                         (self.CONST_PST_MAX - self.CONST_PST_MIN)) * \
                (self._freq_max - self._freq_min) + self._freq_min

            self._amplitudes[finger] = max(self._amplitudes[finger], self.fading_amplitudes[finger])
            self._frequencies[finger] = max(self._frequencies[finger], self.fading_frequencies[finger])

            self._prev_values[finger] = mapped_values[finger]

    def _pst_tactile_cb(self, data):
        if len(data.pressure) == len(self.CONST_FINGERS):
            for i, press in enumerate(data.pressure):
                self._normalized_pressure[i] = (press - self._pst_threshold[i]) / \
                                         (self._pst_saturation[i] - self._pst_threshold[i])
                self._normalized_pressure[i] = min(max(self._normalized_pressure[i], 0), 1)
            self._process_tactile_data(dict(zip(self.CONST_FINGERS, self._normalized_pressure)))
        else:
            rospy.logwarn("Missing data. Expected to receive {}, but got {} PST values".format(len(self.CONST_FINGERS),
                                                                                               len(data.pressure)))

    def _biotac_tactile_cb(self, data):
        if len(data.movables) == len(self.CONST_FINGERS):
            if self._hand_id in data.movables[0].name:
                for i in range(0, len(data.movables)):
                    self._normalized_pressure[i] = data.movables[i].tactors[0].pressure
            self._process_tactile_data(dict(zip(self.CONST_FINGERS, self._normalized_pressure)))
        else:
            rospy.logwarn("Missing data. Expected to receive {}, "
                          "but got {} Biotac values".format(len(self.CONST_FINGERS), len(self._normalized_pressure)))

    def _reconfigure(self, config, level):
        self._contact_time = config.contact_time
        self._amp_max = config.max_amplitude
        self._amp_min = config.min_amplitude
        self._freq_max = config.max_frequency
        self._freq_min = config.min_frequency
        self._pst_saturation = len(self.CONST_FINGERS) * [config.pst_saturation]
        rospy.logwarn("reconfigured")
        return config


if __name__ == "__main__":

    rospy.init_node('sr_finger_mount_node')

    fingers = rospy.get_param("~fingers", 'th')
    hand_id = rospy.get_param("~hand_id", 'rh')

    if not (hand_id == "rh" or hand_id == "lh"):
        raise ValueError('/hand_id is not rh or lh')

    if fingers is not None:
        fingers = fingers.split(',')

    mount = SrPiezoFeedback(fingers, hand_id)
    # srv = Server(SrFingerMountConfig, mount._reconfigure)
    rospy.spin()
