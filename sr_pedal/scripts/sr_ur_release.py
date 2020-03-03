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

import rospy
from sr_pedal.msg import Status
from ur_dashboard_msgs.srv import GetSafetyMode, GetProgramState, GetRobotMode, Load
from ur_dashboard_msgs.msg import SafetyMode, ProgramState, RobotMode
from std_srvs.srv import Trigger

class SrUrUnlock():
    def __init__(self):
        self.arms = []
        if rospy.has_param('ra_sr_ur_robot_hw'):
            self.arms.append('ra')
        if rospy.has_param('la_sr_ur_robot_hw'):
            self.arms.append('la')
        if len(self.arms) == 0:
            rospy.signal_shutdown("No arms detected, shutting down %s", rospy.get_name())

        self.subscriber = rospy.Subscriber("sr_pedal/status", Status, self.pedal_sub)
        self.rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("Shutting down %s", rospy.get_name())

    def pedal_sub(self, data):
        if data.connected and data.left_pressed and data.right_pressed:
            self.release_arm()

    def call_dashboard_service(self, side, service_name):
        service_string = "/" + side + "_sr_ur_robot_hw/dashboard/" + service_name
        serv_call = rospy.ServiceProxy(service_string, Trigger)
        resp = serv_call()

    def get_safety_mode(self, arm):
        safety_mode_service = rospy.ServiceProxy("/" + arm + "_sr_ur_robot_hw/dashboard/get_safety_mode", GetSafetyMode)
        safety_mode_msg = safety_mode_service()
        return safety_mode_msg.safety_mode.mode

    def get_robot_mode(self, arm):
        get_mode_service = rospy.ServiceProxy("/" + arm + "_sr_ur_robot_hw/dashboard/get_robot_mode", GetRobotMode)
        get_mode_msg = get_mode_service()
        return get_mode_msg.robot_mode.mode

    def startup_arm(self, arm):
        self.call_dashboard_service(arm, "power_on")
        rospy.sleep(5)
        self.call_dashboard_service(arm, "brake_release")
        rospy.sleep(5)

    def load_external_control_program(self, arm):
        serv_call = rospy.ServiceProxy("/" + arm + "_sr_ur_robot_hw/dashboard/load_program", Load)
        resp = serv_call("external_ctrl.urp")
        print resp

    def release_arm(self):
        for arm in self.arms:
            rospy.loginfo("Checking arm: %s", arm)
            rospy.wait_for_service("/" + arm + "_sr_ur_robot_hw/dashboard/get_safety_mode")
            try:
                safety_mode = self.get_safety_mode(arm)
                if safety_mode == SafetyMode.ROBOT_EMERGENCY_STOP:
                    rospy.logwarn("Emergency stop button is still pressed for arm %s, please release", arm)
                    while safety_mode == SafetyMode.ROBOT_EMERGENCY_STOP:
                        rospy.logwarn("Emergency stop button is still pressed for arm %s, please release", arm)
                        safety_mode = self.get_safety_mode(arm)
                        rospy.sleep(2)
                if safety_mode == SafetyMode.PROTECTIVE_STOP:
                    rospy.loginfo("Protective stop detected on: %s", arm)
                    self.call_dashboard_service(arm, "unlock_protective_stop")
                if safety_mode == SafetyMode.FAULT:
                    rospy.loginfo("Fault detected on: %s", arm)
                    self.call_dashboard_service(arm, "restart_safety")
                    rospy.sleep(15)
                safety_mode = self.get_safety_mode(arm)
                if safety_mode == SafetyMode.PROTECTIVE_STOP:
                    rospy.loginfo("Protective stop detected on: %s", arm)
                    self.call_dashboard_service(arm, "unlock_protective_stop")
                self.call_dashboard_service(arm, "close_safety_popup")
                rospy.sleep(2)
                self.call_dashboard_service(arm, "close_popup")
                rospy.sleep(2)
                robot_mode = self.get_robot_mode(arm)
                if robot_mode == RobotMode.IDLE or robot_mode == RobotMode.POWER_OFF:
                    rospy.loginfo("Starting arm: %s", arm)
                    self.startup_arm(arm)
                play_mode_service = rospy.ServiceProxy("/" + arm + "_sr_ur_robot_hw/dashboard/program_state", GetProgramState)
                play_msg = play_mode_service()
                if play_msg.program_name == "null":
                    rospy.loginfo("Loading program: %s for arm: %s", play_msg.program_name, arm)
                    self.load_external_control_program(arm)
                    rospy.sleep(2)
                if play_msg.state.state == ProgramState.STOPPED or play_msg.state.state == ProgramState.PAUSED:
                    rospy.sleep(5)
                    rospy.loginfo("Starting program: %s for arm: %s", play_msg.program_name, arm)
                    self.call_dashboard_service(arm, "play")
            except rospy.ServiceException:
                print "Service call failed for arm: " + arm 

if __name__ == "__main__":
    rospy.init_node("sr_ur_unlock_node")
    sr_ur_unlock = SrUrUnlock()
