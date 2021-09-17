#!/usr/bin/env python
# Copyright 2020, 2021 Shadow Robot Company Ltd.
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

import rospy
import requests
import os
import yaml
import rospkg
import subprocess
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry
import sr_teleop_manager.msg
from sr_teleop_manager.srv import CustomRelayCommand, CustomRelayCommandResponse


class PowerControlCommon(object):
    _on_off_delay = 0.4

    def check_relays_connected(self):
        for device in self._devices:
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
        if b'fping: command not found' in stdout:
            rospy.logerr("fping not installed, please install fping")
        if p.returncode == 0:
            return True
        else:
            return False

    def does_ip_relay_respond(self, ip):
        return self.is_ping_successful(ip)


class RemotePowerControl(PowerControlCommon):
    def __init__(self, devices):
        self._devices = devices
        self._custom_relay_service = rospy.Service('custom_power_relay_command', CustomRelayCommand,
                                                   self.handle_custom_relay_request)
        self.check_relays_connected()
        self.print_config()
        self.set_relays_to_default()
        while not rospy.is_shutdown():
            rospy.spin()

    def print_config(self):
        rospy.loginfo("Devices in config:")
        for device in self._devices:
            rospy.loginfo("Name: %s", device['name'])
            for key, value in device.items():
                if 'name' != key:
                    rospy.loginfo("  %s: %s", key, value)

    def handle_custom_relay_request(self, req):
        request_string = 'http://' + str(req.ip_address) + '/' + req.command_string
        response = self.requests_retry_session().get(request_string)
        response_content = response.content.replace("<html><body>", '').replace("</body></html>\r\n\r\n", '')
        return CustomRelayCommandResponse(response_content)


if __name__ == "__main__":
    rospy.init_node('remote_power_control', anonymous=False)
    config_file = 'power_devices.yaml'
    config_path = os.path.join(rospkg.RosPack().get_path('sr_teleop_manager'), 'config')
    config = os.path.join(config_path, config_file)
    with open(config) as f:
        dataMap = yaml.safe_load(f)
        
    remote_power_control = RemotePowerControl(dataMap)
