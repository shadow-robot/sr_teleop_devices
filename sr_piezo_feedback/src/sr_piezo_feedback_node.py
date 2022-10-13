#!/usr/bin/env python3

# Copyright 2021, 2022 Shadow Robot Company Ltd.
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

from __future__ import absolute_import, division

import math
import threading
import time

import numpy as np
import rospy
import sounddevice as sd
from dynamic_reconfigure.server import Server
from sr_hand.tactile_receiver import TactileReceiver
from sr_robot_msgs.msg import BiotacAll, ShadowPST
from sr_piezo_feedback.msg import PiezoFeedback


class DeviceHandler(threading.Thread):
    def __init__(self, device, fingers, mount):
        super(__class__, self).__init__()
        self._device_name = device
        self._finger_per_devices = fingers
        self._mount = mount
        self._samplerate = sd.query_devices(self._device_name, 'output')['default_samplerate']

        self._ts = [0] * len(self._finger_per_devices)
        self._oldsignal = [0] * len(self._finger_per_devices)
        self._freq = [1] * len(self._finger_per_devices)
        self._amp = [1] * len(self._finger_per_devices)
        self._signal_range = [-0.1, 0.6]  # specific values for the Boreas DevKit
        self._offset = np.mean(self._signal_range)
        self._amp_factor = np.mean(np.abs(self._signal_range))

        self._publisher = dict(zip(self._finger_per_devices, len(self._finger_per_devices) * [0]))
        self._finger_msg = dict(zip(self._finger_per_devices, len(self._finger_per_devices) * [0]))

        for finger in self._finger_per_devices:
            self._publisher[finger] = rospy.Publisher('sr_piezo_feedback/'+finger, PiezoFeedback, queue_size=1)
            self._finger_msg[finger] = PiezoFeedback()
            self._finger_msg[finger].feedback.data = [float(self._oldsignal[0]), float(self._amp[0]),
                                                      float(self._freq[0])]

    def run(self):
        self.start_piezo()

    def callback(self, outdata, frames, _time, status):
        if status:
            rospy.logwarn(status)

        for i, finger in enumerate(self._finger_per_devices):
            for frame in range(frames):
                outdata[frame, i] = self._offset + self._amp_factor * self._amp[i] * np.sin(self._ts[i])
                self._ts[i] += 2 * np.pi * self._freq[i] / self._samplerate

                if outdata[frame, i] != np.sign(self._oldsignal[i]):
                    self._freq[i] = self._mount.frequencies[finger]
                    self._amp[i] = self._mount.amplitudes[finger]
                self._oldsignal[i] = outdata[frame, i]

                self._finger_msg[finger].feedback.data = [float(outdata[frame, i]), float(self._amp[i]),
                                                          float(self._freq[i])]
                self._publisher[finger].publish(self._finger_msg[finger])

    def start_piezo(self):
        with sd.OutputStream(device=self._device_name, channels=2, callback=self.callback,
                             samplerate=self._samplerate, blocksize=100, latency='low'):
            while not rospy.is_shutdown():
                rospy.sleep(0.1)


class SrPiezoFeedback():

    CONST_TACTILE_MAX = 1
    CONST_TACTILE_MIN = 0

    CONST_TOUCH_DETECTION_THRESHOLD = 0.03  # Defines sensor value range from 0 to CONST_TOUCH_DETECTION_THRESHOLD
    # which is considered as no contact.
    CONST_TOUCH_STRENGTH_DIFFERENCE = 0.02  # Defines the sensitivity for the contact detection. Higher value increases
    # sensitivity, lower decreases. This values is compared to the difference between current and past sensor readings

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
        self.amplitudes = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self.frequencies = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))

        self._device_handlers = [None, None, None]

        # TO BE REMOVED IN FUTURE
        self._start_time = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self.fading_time = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self.fading_amplitudes = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))
        self.fading_frequencies = dict(zip(self.CONST_FINGERS, len(self.CONST_FINGERS) * [0]))

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

        if not set(self._used_fingers).intersection(set(self.CONST_FINGERS)):
            rospy.logerr("Failed to start node! Used fingers are not allowed: {}".format(self.CONST_FINGERS))
            return
        if not self._check_devices():
            return

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

            if self._prev_values[finger] < self.CONST_TOUCH_DETECTION_THRESHOLD and \
               np.abs(mapped_values[finger]-self._prev_values[finger]) > self.CONST_TOUCH_STRENGTH_DIFFERENCE:
                self._start_time[finger] = rospy.get_time()

            # REMOVE IN FUTURE
            if self.fading_time[finger] <= self._contact_time:
                fading = -4 / (self._contact_time * self._contact_time)
                fading_factor = fading * self.fading_time[finger] * \
                                self.fading_time[finger] + \
                                fading * self.fading_time[finger]
                self.fading_amplitudes[finger] = self._amp_max * 1.0 * fading_factor
                self.fading_frequencies[finger] = self._freq_max * fading_factor
            else:
                self.fading_amplitudes[finger] = 0
                self.fading_frequencies[finger] = 0

            self.amplitudes[finger] = ((mapped_values[finger] - self.CONST_TACTILE_MIN) /
                                        (self.CONST_TACTILE_MAX - self.CONST_TACTILE_MIN)) * \
                (self._amp_max - self._amp_min) + self._amp_min
            self._frequencies[finger] = ((mapped_values[finger] - self.CONST_TACTILE_MIN) /
                                         (self.CONST_TACTILE_MAX - self.CONST_TACTILE_MIN)) * \
                (self._freq_max - self._freq_min) + self._freq_min

            self.amplitudes[finger] = max(self.amplitudes[finger], self.fading_amplitudes[finger])
            self.frequencies[finger] = max(self.frequencies[finger], self.fading_frequencies[finger])

            self._prev_values[finger] = mapped_values[finger]

    @staticmethod
    def mapping(value, threshold, saturation, exponent=1.0, out_min=0.0, out_max=1.0):
        out = 0.0
        try:
            out = (value - threshold)/(saturation-threshold)
            out = min(max(out, out_min), out_max) ** exponent
        except ZeroDivisionError:
            rospy.logerr("Saturation can't equal threshold. Assuming default value ({}) as output!".format(out))
        return out


class SrPiezoFeedbackPST(SrPiezoFeedback):

    def __init__(self, total_fingers, hand):
        super().__init__(total_fingers, hand)
        self._pst_saturation = [550.0, 550.0, 550.0, 550.0, 550.0]
        self._init_thresholds()
        rospy.Subscriber("/"+self._hand_id+"/tactile", ShadowPST, self._pst_tactile_cb)
        self.initialize()

    def _init_thresholds(self):
        samples_to_collect = 50
        thresholds_to_set = samples_to_collect * [None]
        for i in range(0, samples_to_collect):
            data = rospy.wait_for_message("/"+self._hand_id+"/tactile", ShadowPST)
            thresholds_to_set[i] = [data.pressure]
        self._pst_threshold = np.mean(thresholds_to_set, axis=1)[0]

    def _pst_tactile_cb(self, data):
        if len(data.pressure) == len(self.CONST_FINGERS):
            for i, press in enumerate(data.pressure):
                self._normalized_pressure[i] = self.mapping(press, self._pst_threshold[i], self._pst_saturation[i])
            self._process_tactile_data(dict(zip(self.CONST_FINGERS, self._normalized_pressure)))
        else:
            rospy.logwarn("Missing data. Expected to receive {}, but got {} PST values".format(len(self.CONST_FINGERS),
                                                                                               len(data.pressure)))

    def reconfigure(self, config):
        self._contact_time = config.contact_time
        self._amp_max = config.max_amplitude
        self._amp_min = config.min_amplitude
        self._freq_max = config.max_frequency
        self._freq_min = config.min_frequency
        self._pst_saturation = len(self.CONST_FINGERS) * [config.pst_saturation]
        return config


class SrPiezoFeedbackBiotac(SrPiezoFeedback):
    def __init__(self, total_fingers, hand):
        super().__init__(total_fingers, hand)
        self._tactile_pdc_ref = len(self.CONST_FINGERS) * [0]
        self._tactile_pac_ref = len(self.CONST_FINGERS) * [0]
        self._pdc_threshold = 5.0
        self._pdc_saturation = 100.0
        self._pdc_mapping_exponent = 2.0
        self._pdc_output_weight = 1.0
        self._pac_threshold = 10.0
        self._pac_saturation = 2000.0
        self._pac_mapping_exponent = 2.0
        self._pac_output_weight = 0.3

        biotac_reference_data = rospy.wait_for_message("/"+self._hand_id+"/tactile", BiotacAll)
        for i, tactile in enumerate(biotac_reference_data.tactiles):
            self._tactile_pdc_ref[i] = tactile.pdc
            self._tactile_pac_ref[i] = np.mean(tactile.pac)

        rospy.Subscriber("/"+self._hand_id+"/tactile", BiotacAll, self._biotac_tactile_cb)
        self.initialize()

    def _process_biotac_tactile_msg(self, data):
        pressure = len(self.CONST_FINGERS) * [0]
        for i, tactile in enumerate(data.tactiles):
            pdc_norm = tactile.pdc - self._tactile_pdc_ref[i]
            pac_norm = np.mean(tactile.pac) - self._tactile_pac_ref[i]
            pressure[i] = [pdc_norm, pac_norm]
        return pressure

    def _biotac_tactile_cb(self, data):
        processed_biotac_pressure = self._process_biotac_tactile_msg(data)
        if len(processed_biotac_pressure) == len(self.CONST_FINGERS):
            for i, _press in enumerate(processed_biotac_pressure):
                pdc = processed_biotac_pressure[i][0]
                pac = processed_biotac_pressure[i][1]
                pdc_out = self.mapping(pdc, self._pdc_threshold, self._pdc_saturation,
                                       exponent=self._pdc_mapping_exponent)
                pac_out = self.mapping(pac, self._pac_threshold, self._pac_saturation,
                                       exponent=self._pac_mapping_exponent)
                self._normalized_pressure[i] = self._pdc_output_weight * pdc_out + self._pac_output_weight * pac_out
            self._process_tactile_data(dict(zip(self.CONST_FINGERS, self._normalized_pressure)))
        else:
            rospy.logwarn(f"Missing data. Expected to receive {len(self.CONST_FINGERS)}, "
                          f"but got {len(processed_biotac_pressure)} Biotac values")

    def reconfigure(self, config):
        self._contact_time = config.contact_time
        self._amp_max = config.max_amplitude
        self._amp_min = config.min_amplitude
        self._freq_max = config.max_frequency
        self._freq_min = config.min_frequency
        self._pdc_threshold = config.pdc_threshold
        self._pdc_saturation = config.pdc_saturation
        self._pdc_mapping_exponent = config.pdc_mapping_exponent
        self._pac_threshold = config.pac_threshold
        self._pac_saturation = config.pac_saturation
        self._pac_mapping_exponent = config.pac_mapping_exponent
        self._pac_output_weight = config.pac_output_weight
        self._pdc_output_weight = config.pdc_output_weight
        return config


if __name__ == "__main__":

    rospy.init_node('sr_finger_mount_node')

    fingers_param = rospy.get_param("~fingers", 'th')
    hand_id_param = rospy.get_param("~hand_id", 'rh')

    if hand_id_param not in ("rh", "lh"):
        raise ValueError('/hand_id is not rh or lh')

    if fingers_param is not None:
        fingers_list = fingers_param.split(',')

    tactile_type = TactileReceiver(hand_id_param).get_tactile_type()
    if tactile_type == "PST":
        pst_feedback = SrPiezoFeedbackPST(fingers_list, hand_id_param)
        from sr_piezo_feedback.cfg import SrPiezoFeedbackPSTConfig
        srv = Server(SrPiezoFeedbackPSTConfig, pst_feedback.reconfigure)
    elif tactile_type == "biotac":
        biotac_feedback = SrPiezoFeedbackBiotac(fingers_list, hand_id_param)
        from sr_piezo_feedback.cfg import SrPiezoFeedbackBiotacConfig
        srv = Server(SrPiezoFeedbackBiotacConfig, biotac_feedback.reconfigure)

    rospy.spin()
