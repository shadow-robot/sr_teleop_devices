#!/usr/bin/env python
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import requests
import actionlib
import os
import yaml
import rospkg
import subprocess
import threading
import paramiko
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
import sr_teleop_manager.msg
from sr_teleop_manager.msg import PowerManagerAction
from sr_teleop_manager.msg import BootProgress
from sr_teleop_manager.srv import CustomRelayCommand, CustomRelayCommandResponse


class PowerControlCommon(object):
    _on_off_delay = 0.4

    def check_relays_connected(self):
        for device in self._devices:
            if 'arm' in device['name']:
                if self.does_ip_relay_respond(device['power_ip']):
                    rospy.loginfo("Contacted " + device['name'] + " ip relay at " + device['power_ip'])
                else:
                    rospy.logwarn("Could not contact " + device['name'] + " ip relay at " + device['power_ip'])

    def set_relays_to_default(self):
        for device in self._devices:
            device_ip = device['power_ip']
            if self.does_ip_relay_respond(device_ip):
                response = self.requests_retry_session().get('http://' + device_ip + '/setpara[45]=0')

    def requests_retry_session(self, retries=3, backoff_factor=0.3, status_forcelist=(500, 502, 504), session=None):
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

    def is_ping_successful(self, ip):
        command = 'fping -c1 -t500 ' + ip + ' 2>&1 >/dev/null'
        p = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
        stdout = p.communicate()[0]
        if 'fping: command not found' in stdout:
            rospy.logerr("fping not installed, please install fping")
        if p.returncode == 0:
            return True
        else:
            return False

    def does_ip_relay_respond(self, ip):
        return self.is_ping_successful(ip)

    def is_arm_off(self, ip):
        return not self.is_ping_successful(ip)

    def is_arm_on(self, ip):
        return self.is_ping_successful(ip)

    def power_on(self, power_ip):
        responses = []
        response = self.requests_retry_session().get('http://' + power_ip + '/setpara[45]=1')
        responses.append(response)
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get('http://' + power_ip + '/setpara[45]=0')
        responses.append(response)
        rospy.sleep(self._on_off_delay)
        return responses

    def power_off(self, power_ip):
        responses = []
        response = self.requests_retry_session().get('http://' + power_ip + '/setpara[46]=1')
        responses.append(response)
        rospy.sleep(self._on_off_delay)
        response = self.requests_retry_session().get('http://' + power_ip + '/setpara[46]=0')
        responses.append(response)
        rospy.sleep(self._on_off_delay)
        return responses


class RemotePowerControl(PowerControlCommon):
    _feedback = sr_teleop_manager.msg.PowerManagerFeedback()
    _result = sr_teleop_manager.msg.PowerManagerResult()

    def __init__(self, name, devices):
        self._action_name = name
        self._devices = devices
        self._CONST_ALL_THREADS_FINISHED_TIMEOUT = 200
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

    def print_config(self):
        rospy.loginfo("Devices in config:")
        for device in self._devices:
            rospy.loginfo("Name: %s", device['name'])
            for key, value in device.iteritems():
                if 'name' != key:
                    rospy.loginfo("  %s: %s", key, value)

    def handle_custom_relay_request(self, req):
        if 'get' in req.request_type.lower():
            req_type = 'get'
        if 'set' in req.request_type.lower():
            req_type = 'set'
        if req.command_string == "":
            request_string = 'http://' + str(req.ip_address) + '/' + str(req_type) + 'para[' + str(
                req.param_number) + ']=' + str(req.value)
        else:
            request_string = 'http://' + str(req.ip_address) + '/' + req.command_string
        response = self.requests_retry_session().get(request_string)
        response_content = response.content.replace("<html><body>", '').replace("</body></html>\r\n\r\n", '')
        return CustomRelayCommandResponse(response_content)

    def execute_cb(self, goal):
        rospy.loginfo('%s: Executing. \nPower on list: %s \nPower off list: %s' % (
                        self._action_name, goal.power_on, goal.power_off))
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
        while any(thread.is_alive() for name, thread in threads.iteritems()):
            if (now.secs + self._CONST_ALL_THREADS_FINISHED_TIMEOUT) < rospy.get_rostime().secs:
                break
        rospy.loginfo("All goal tasks processed")

        if not any(thread.is_alive() for name, thread in threads.iteritems()):
            self._as.set_succeeded(self._result)
        else:
            rospy.logerr("something went wrong, a thread hasn't returned")
            for name, thread in threads.iteritems():
                if thread.is_alive():
                    rospy.logerr("thread(s) still alive: %s", name)

        # Reset _feedback and _result for next goal
        self._feedback = sr_teleop_manager.msg.PowerManagerFeedback()
        self._result = sr_teleop_manager.msg.PowerManagerResult()


class BootMonitor(threading.Thread, PowerControlCommon):
    def __init__(self, thread_id, device, action_server, feedback, result, on_off):
        threading.Thread.__init__(self)
        self._on_off = on_off
        self.threadID = thread_id
        self._feedback = feedback
        self._CONST_UR_ARM_SSH_USERNAME = 'root'
        self._CONST_UR_ARM_SSH_PASSWORD = 'easybot'
        self._device_name = device['name']
        if 'arm' in device['name']:
            self._data_ip = device['data_ip']
        self._power_ip = device['power_ip']
        self._as = action_server
        self._CONST_PING_TIMEOUT = 80.0
        self._CONST_GET_LOG_TIMEOUT = 60.0
        self._CONST_FINISH_BOOTING_TIMEOUT = 60.0
        self._result = result

    def check_preempt(self):
        if self._as.is_preempt_requested():
            rospy.loginfo('Preempted: %s' % self._device_name)
            self.add_feedback('Preempted: %s' % self._device_name, boot_status=BootProgress.BOOT_STATUS_BOOT_CANCELED,
                              failed=True, finished=False)
            self._as.set_preempted()
            return True
        return False

    def run(self):
        result = sr_teleop_manager.msg.PowerResult()
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
                rospy.loginfo('%s: Succeeded' % self._device_name)
                return True
            else:
                result.success = False
                self._result.results.append(result)
                rospy.logerr('%s: FAILED' % self._device_name)
                return False
        else:
            return self.boot_hand()

    def shutdown_arm(self):
        if self.is_arm_on(self._data_ip):
            self.add_feedback("shutting down %s" % self._device_name,
                              boot_status=BootProgress.BOOT_STATUS_SHUTTING_DOWN)
            responses = self.power_off(self._power_ip)
            if not all(response.status_code == 200 for response in responses):
                self.add_feedback("Could not contact relay",
                                  boot_status=BootProgress.BOOT_STATUS_BOOT_FAIL_NO_RELAY)
                return False
            now = rospy.get_rostime()
            while self.is_ping_successful(self._data_ip):
                if (rospy.get_rostime().secs + self._CONST_PING_TIMEOUT) < now.secs:
                    rospy.logerr("")
                    self.add_feedback("Failed to shutdown arm", failed=True,
                                      boot_status=BootProgress.BOOT_STATUS_SHUTDOWN_FAILED)
                    return False
            self.add_feedback("Arm shutdown successfully", failed=False, finished=True,
                              boot_status=BootProgress.BOOT_STATUS_SHUTDOWN_SUCCEEDED)
            return True
        else:
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
        else:
            self.add_feedback("failed to boot hand", boot_status=BootProgress.BOOT_STATUS_BOOT_FAILED, failed=True,
                              finished=True)
            return False

    def is_hand_on(self):
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
                if (rospy.get_rostime().secs + self._CONST_PING_TIMEOUT) < now.secs:
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
                if (rospy.get_rostime().secs + self._CONST_GET_LOG_TIMEOUT) < now.secs:
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
                if (rospy.get_rostime().secs + self._CONST_FINISH_BOOTING_TIMEOUT) < now.secs:
                    rospy.logerr("Failed to finish booting, arm boot failed")
                    self.add_feedback("arm boot time out, failed", failed=True, finished=True,
                                      boot_status=BootProgress.BOOT_STATUS_BOOT_FAILED)
                    return False
            self.add_feedback("Finished booting arm!", finished=True,
                              boot_status=BootProgress.BOOT_STATUS_BOOT_SUCCESS)
            return True
        else:
            self.add_feedback("Arm already on", failed=True, finished=True,
                              boot_status=BootProgress.BOOT_STATUS_REDUNDANT_REQUEST)
            return False

    def add_feedback(self, status_message, boot_status, finished=False, failed=False):
        rospy.loginfo("%s: %s", self._device_name, status_message)
        fb = sr_teleop_manager.msg.PowerFeedback()
        fb.status = self._device_name + " " + status_message
        fb.name = self._device_name
        fb.complete = finished
        bp = BootProgress()
        bp.boot_status = boot_status
        fb.boot_status = bp
        fb.failed = failed
        threadLock.acquire()
        self._feedback.feedback.append(fb)
        self._as.publish_feedback(self._feedback)
        threadLock.release()

    def get_log_from_arm(self, arm_ip):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(arm_ip, username=self._CONST_UR_ARM_SSH_USERNAME, password=self._CONST_UR_ARM_SSH_PASSWORD)
        stdin, stdout, stderr = client.exec_command('cat /tmp/log/urcontrol/current')
        log = stdout.readlines()
        client.close()
        return log


threadLock = threading.Lock()

if __name__ == "__main__":
    rospy.init_node('remote_power_control', anonymous=False)
    config_file = 'power_devices.yaml'
    config_path = os.path.join(rospkg.RosPack().get_path('sr_teleop_manager'), 'config')
    config = os.path.join(config_path, config_file)
    with open(config) as f:
        dataMap = yaml.safe_load(f)

    remote_power_control = RemotePowerControl(rospy.get_name(), dataMap)
