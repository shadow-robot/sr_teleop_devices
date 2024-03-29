#!/usr/bin/env python
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

import os
import subprocess
import threading
from abc import ABC
import actionlib
import paramiko
import requests
import rospkg
import rospy
import yaml
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
from sr_teleop_manager.msg import (BootProgress,
                                   PowerManagerAction,
                                   PowerManagerFeedback,
                                   PowerManagerResult,
                                   PowerResult,
                                   PowerFeedback)
from sr_teleop_manager.srv import (CustomRelayCommand,
                                   CustomRelayCommandResponse)


class PowerControlCommon(ABC):
    _on_off_delay = 0.4

    @devices.setter
    def devices(self, value):
        self._devices = value

    def check_relays_connected(self):
        for device in self._devices:
            if 'arm' in device['name']:
                if self.does_ip_relay_respond(device['power_ip']):
                    rospy.loginfo(f"Contacted {device['name']} ip relay at {device['power_ip']}")
                else:
                    rospy.logwarn(f"Could not contact {device['name']} ip relay at {device['power_ip']}")

    def set_relays_to_default(self):
        for device in self._devices:
            device_ip = device['power_ip']
            if self.does_ip_relay_respond(device_ip):
                self.requests_retry_session().get(f"http://{device_ip}/setpara[45]=0")

    @staticmethod
    def requests_retry_session(retries=3, backoff_factor=0.3, status_forcelist=(500, 502, 504), session=None):
        session = session or requests.Session()
        retry = Retry(
            total=retries,
            read=retries,
            connect=retries,
            backoff_factor=backoff_factor,
            status_forcelist=status_forcelist,
        )
        adapter = HTTPAdapter(max_retries=retry)
        session.mount('http://', adapter)
        session.mount('https://', adapter)
        return session

    @staticmethod
    def is_ping_successful(ping_ip):
        command = f"fping -c1 -t500 {ping_ip} 2>&1 >/dev/null"
        with subprocess.Popen(command, stdout=subprocess.PIPE, shell=True) as process:
            stdout = process.communicate()[0]
            if 'fping: command not found' in stdout:
                rospy.logerr("fping not installed, please install fping")
            if process.returncode == 0:
                return True

        return False

    def does_ip_relay_respond(self, ip_relay):
        return self.is_ping_successful(ip_relay)

    def is_arm_off(self, arm_ip):
        return not self.is_ping_successful(arm_ip)

    def is_arm_on(self, arm_ip):
        return self.is_ping_successful(arm_ip)

    def power_on(self, power_ip):
        responses = []
        response = self.requests_retry_session().get(f'http://{power_ip}/setpara[45]=1')
        responses.append(response)
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get(f'http://{power_ip}/setpara[45]=0')
        responses.append(response)
        rospy.sleep(self._on_off_delay)
        return responses

    def power_off(self, power_ip):
        responses = []
        response = self.requests_retry_session().get(f'http://{power_ip}/setpara[46]=1')
        responses.append(response)
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get(f'http://{power_ip}/setpara[46]=0')
        responses.append(response)
        rospy.sleep(self._on_off_delay)
        return responses


class RemotePowerControl(PowerControlCommon):
    _feedback = PowerManagerFeedback()
    _result = PowerManagerResult()

    def __init__(self, name, devices):
        self._action_name = name
        self.devices(devices)
        self._all_threads_finished_timeout = 200
        self._as = actionlib.SimpleActionServer(self._action_name, PowerManagerAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._custom_relay_service = rospy.Service('custom_power_relay_command', CustomRelayCommand,
                                                   self.handle_custom_relay_request)
        self._as.start()
        self.check_relays_connected()
        self.print_config()
        self.set_relays_to_default()
        while not rospy.is_shutdown():
            rospy.spin()

    @devices.setter
    def devices(self, value):
        self._devices = value

    def print_config(self):
        rospy.loginfo("Devices in config:")
        for device in self._devices:
            rospy.loginfo(f"Name: {device['name']}")
            for key, value in device.items():
                if key != 'name':
                    rospy.loginfo(f"  {key}: {value}")

    def handle_custom_relay_request(self, req):
        if 'get' in req.request_type.lower():
            req_type = 'get'
        if 'set' in req.request_type.lower():
            req_type = 'set'
        if req.command_string == "":
            request_string = (f"http://{req.ip_address}/{req_type}para["
                              f"{req.param_number}]={req.value}")
        else:
            request_string = f"http://{req.ip_address}/{req.command_string}"
        response = self.requests_retry_session().get(request_string)
        response_content = response.content.replace("<html><body>", '').replace("</body></html>\r\n\r\n", '')
        return CustomRelayCommandResponse(response_content)

    def execute_cb(self, goal):
        rospy.loginfo(f'{self._action_name}: Executing. \nPower on list: {goal.power_on} \n'
                      f'Power off list: {goal.power_off}')
        threads = {}
        thread_id_counter = 1
        for power_on_goal in goal.power_on:
            for device in self._devices:
                if device['name'] == power_on_goal:
                    rospy.loginfo("powering on...")
                    threads[device['name']] = BootMonitor(thread_id_counter, device, self._as,
                                                          self._feedback, self._result, "on")
                    thread_id_counter = thread_id_counter + 1
                    threads[device['name']].start()

        for power_off_goal in goal.power_off:
            for device in self._devices:
                if device['name'] == power_off_goal:
                    rospy.loginfo("powering off...")
                    threads[device['name']] = BootMonitor(thread_id_counter, device, self._as,
                                                          self._feedback, self._result, "off")
                    thread_id_counter = thread_id_counter + 1
                    threads[device['name']].start()

        now = rospy.get_rostime()
        rospy.logwarn("waiting for threads to complete...")
        while any(thread.is_alive() for name, thread in threads.items()):
            if (now.secs + self._all_threads_finished_timeout) < rospy.get_rostime().secs:
                break
        rospy.loginfo("All goal tasks processed")

        if not any(thread.is_alive() for name, thread in threads.items()):
            self._as.set_succeeded(self._result)
        else:
            rospy.logerr("something went wrong, a thread hasn't returned")
            for name, thread in threads.items():
                if thread.is_alive():
                    rospy.logerr("thread(s) still alive: %s", name)

        # Reset _feedback and _result for next goal
        self._feedback = PowerManagerFeedback()
        self._result = PowerManagerResult()


class BootMonitor(threading.Thread, PowerControlCommon):
    def __init__(self, thread_id, device, action_server, feedback, result, on_off):
        threading.Thread.__init__(self)
        self._on_off = on_off
        self.thread_id = thread_id
        self._feedback = feedback
        self._ur_arm_ssh_username = 'root'
        self._ur_arm_ssh_password = 'easybot'
        self._device_name = device['name']
        if 'arm' in device['name']:
            self._data_ip = device['data_ip']
        self._power_ip = device['power_ip']
        self._as = action_server
        self._ping_timeout = 80.0
        self._get_log_timeout = 60.0
        self._finish_booting_timeout = 60.0
        self._result = result

    def check_preempt(self):
        if self._as.is_preempt_requested():
            rospy.loginfo(f'Preempted: {self._device_name}')
            self.add_feedback(f'Preempted: {self._device_name}', boot_status=BootProgress.BOOT_STATUS_BOOT_CANCELED,
                              failed=True, finished=False)
            self._as.set_preempted()
            return True
        return False

    def run(self):
        result = PowerResult()
        result.name = self._device_name
        result.command = self._on_off
        if 'arm' in self._device_name:
            if self._on_off == 'on':
                success = self.boot_arm()
            else:
                success = self.shutdown_arm()
            if success:
                result.success = True
                self._result.results.append(result)
                rospy.loginfo(f'{self._device_name}: Succeeded')
                return True
            result.success = False
            self._result.results.append(result)
            rospy.logerr(f'{self._device_name}: FAILED')
            return False
        return self.boot_hand()

    def shutdown_arm(self):
        if self.is_arm_on(self._data_ip):
            self.add_feedback(f"shutting down {self._device_name}",
                              boot_status=BootProgress.BOOT_STATUS_SHUTTING_DOWN)
            responses = self.power_off(self._power_ip)
            if not all(response.status_code == 200 for response in responses):
                self.add_feedback("Could not contact relay",
                                  boot_status=BootProgress.BOOT_STATUS_BOOT_FAIL_NO_RELAY)
                return False
            now = rospy.get_rostime()
            while self.is_ping_successful(self._data_ip):
                if (rospy.get_rostime().secs + self._ping_timeout) < now.secs:
                    rospy.logerr("")
                    self.add_feedback("Failed to shutdown arm", failed=True,
                                      boot_status=BootProgress.BOOT_STATUS_SHUTDOWN_FAILED)
                    return False
            self.add_feedback("Arm shutdown successfully", failed=False, finished=True,
                              boot_status=BootProgress.BOOT_STATUS_SHUTDOWN_SUCCEEDED)
            return True
        self.add_feedback("Arm already off", failed=True, finished=True,
                          boot_status=BootProgress.BOOT_STATUS_REDUNDANT_REQUEST)
        return False

    # This function will be corrected when we have hand power control
    def boot_hand(self):
        self.add_feedback("booting hand...", boot_status=BootProgress.BOOT_STATUS_OFF)
        self.power_hand_on()
        if self.is_hand_on():
            self.add_feedback("booted!", boot_status=BootProgress.BOOT_STATUS_BOOT_SUCCESS)
            return True
        self.add_feedback("failed to boot hand", boot_status=BootProgress.BOOT_STATUS_BOOT_FAILED, failed=True,
                          finished=True)
        return False

    @staticmethod
    def is_hand_on():
        return True

    def power_hand_on(self):
        pass

    def boot_arm(self):
        if self.is_arm_off(self._data_ip):
            self.add_feedback("was off, now booting...", boot_status=BootProgress.BOOT_STATUS_OFF)
            responses = self.power_on(self._power_ip)
            if not all(response.status_code == 200 for response in responses):
                self.add_feedback("Could not contact relay", boot_status=BootProgress.BOOT_STATUS_BOOT_FAIL_NO_RELAY)
                return False
            now = rospy.get_rostime()
            self.add_feedback("waiting for ping to succeed", boot_status=BootProgress.BOOT_STATUS_PINGING)
            while not self.is_ping_successful(self._data_ip):
                if self.check_preempt():
                    return False
                if (rospy.get_rostime().secs + self._ping_timeout) < now.secs:
                    rospy.logerr("Failed to ping arm, boot failed")
                    self.add_feedback("ping failed, cannot boot arm", failed=True,
                                      boot_status=BootProgress.BOOT_STATUS_BOOT_FAIL_NO_ARM)
                    return False
            self.add_feedback("ping successful, waiting for log to return...",
                              boot_status=BootProgress.BOOT_STATUS_WAITING_FOR_LOG)
            # TODO: also fail nicely here if ping times out
            log = ""
            now = rospy.get_rostime()
            while log == "":
                log = self.get_log_from_arm(self._data_ip)
                self.add_feedback("waiting for log...", boot_status=BootProgress.BOOT_STATUS_WAITING_FOR_LOG)
                if self.check_preempt():
                    return False
                if (rospy.get_rostime().secs + self._get_log_timeout) < now.secs:
                    rospy.logerr("Failed to retrieve log, arm boot failed")
                    self.add_feedback("log retrieval timed out, cannot boot arm", failed=True, finished=True,
                                      boot_status=BootProgress.BOOT_STATUS_BOOT_FAILED)
                    return False
            self.add_feedback("log retrieved", boot_status=BootProgress.BOOT_STATUS_GOT_LOG)
            self.add_feedback("waiting for arm to finish booting...",
                              boot_status=BootProgress.BOOT_STATUS_WAITING_FOR_BOOT_SUCCESS)
            now = rospy.get_rostime()
            log_line = ""
            while "New safety mode: SAFETY_MODE_NORMAL" not in log_line:
                for line in self.get_log_from_arm(self._data_ip):
                    log_line = line
                    if self.check_preempt():
                        return False
                    if "New safety mode: SAFETY_MODE_NORMAL" in log_line:
                        break
                if (rospy.get_rostime().secs + self._finish_booting_timeout) < now.secs:
                    rospy.logerr("Failed to finish booting, arm boot failed")
                    self.add_feedback("arm boot time out, failed", failed=True, finished=True,
                                      boot_status=BootProgress.BOOT_STATUS_BOOT_FAILED)
                    return False
            self.add_feedback("Finished booting arm!", finished=True,
                              boot_status=BootProgress.BOOT_STATUS_BOOT_SUCCESS)
            return True
        self.add_feedback("Arm already on", failed=True, finished=True,
                          boot_status=BootProgress.BOOT_STATUS_REDUNDANT_REQUEST)
        return False

    def add_feedback(self, status_message, boot_status, finished=False, failed=False):
        rospy.loginfo(f"{self._device_name}: {status_message}")
        power_fb = PowerFeedback()
        power_fb.status = f"{self._device_name} {status_message}"
        power_fb.name = self._device_name
        power_fb.complete = finished
        boot_process = BootProgress()
        boot_process.boot_status = boot_status
        power_fb.boot_status = boot_process
        power_fb.failed = failed
        with thread_lock:
            self._feedback.feedback.append(power_fb)
            self._as.publish_feedback(self._feedback)

    def get_log_from_arm(self, arm_ip):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(arm_ip, username=self._ur_arm_ssh_username, password=self._ur_arm_ssh_password)
        _stdin, stdout, _stderr = client.exec_command('cat /tmp/log/urcontrol/current')
        log = stdout.readlines()
        client.close()
        return log


thread_lock = threading.Lock()

if __name__ == "__main__":
    rospy.init_node('remote_power_control', anonymous=False)
    config_file = 'power_devices.yaml'
    config_path = os.path.join(rospkg.RosPack().get_path('sr_teleop_manager'), 'config')
    config = os.path.join(config_path, config_file)
    with open(config, "w", encoding="utf8") as file:
        dataMap = yaml.safe_load(file)

    remote_power_control = RemotePowerControl(rospy.get_name(), dataMap)
