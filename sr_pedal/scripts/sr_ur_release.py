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
        self.external_control_program_name = "external_ctrl.urp"
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

    def call_service(self, side, service_name):
        service_string = "/" + side + "_sr_ur_robot_hw/" + service_name
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

    def startup_arms(self, arms):
        for arm in arms:
            self.call_dashboard_service(arm, "power_on")
        rospy.sleep(5)
        for arm in arms:
            self.call_dashboard_service(arm, "brake_release")
        rospy.sleep(5)

    def load_external_control_program(self, arm):
        serv_call = rospy.ServiceProxy("/" + arm + "_sr_ur_robot_hw/dashboard/load_program", Load)
        resp = serv_call(self.external_control_program_name)
        print resp

    def check_arms_e_stops(self, arms):
        for arm in arms:
            safety_mode = self.get_safety_mode(arm)
            if safety_mode == SafetyMode.ROBOT_EMERGENCY_STOP:
                while safety_mode == SafetyMode.ROBOT_EMERGENCY_STOP:
                    rospy.logwarn("Emergency stop button is still pressed for arm %s, please release", arm)
                    safety_mode = self.get_safety_mode(arm)
                    rospy.sleep(2)

    def check_arms_protective_stop(self, arms):
        for arm in arms:
            safety_mode = self.get_safety_mode(arm)
            if safety_mode == SafetyMode.PROTECTIVE_STOP:
                rospy.loginfo("Protective stop detected on: %s", arm)
                self.call_dashboard_service(arm, "unlock_protective_stop")

    def check_arms_fault(self, arms):
        fault = False
        for arm in arms:
            safety_mode = self.get_safety_mode(arm)
            if safety_mode == SafetyMode.FAULT:
                fault = True
                rospy.loginfo("Fault detected on: %s", arm)
                self.call_dashboard_service(arm, "restart_safety")
        return fault

    def check_arms_robot_mode(self, arms):
        arm_needs_starting = False
        for arm in arms:
            robot_mode = self.get_robot_mode(arm)
            if robot_mode == RobotMode.IDLE or robot_mode == RobotMode.POWER_OFF:
                rospy.loginfo("Starting arm: %s", arm)
                arm_needs_starting = True
        return arm_needs_starting

    def clear_arms_popups(self, arms):
        for arm in arms:
            self.call_dashboard_service(arm, "close_safety_popup")
        rospy.sleep(2)
        for arm in arms:
            self.call_dashboard_service(arm, "close_popup")
        rospy.sleep(2)

    def check_program_loaded_arms(self, arms):
        sleep_time = False
        for arm in arms:
            if not rospy.get_param("/" + arm + "_sr_ur_robot_hw/headless_mode") :
                play_mode_service = rospy.ServiceProxy("/" + arm + "_sr_ur_robot_hw/dashboard/program_state", GetProgramState)
                play_msg = play_mode_service()
                if play_msg.program_name == "null":
                    rospy.loginfo("Not in headless mode. Loading program: %s for arm: %s", self.external_control_program_name, arm)
                    self.load_external_control_program(arm)
                    sleep_time = True
        return sleep_time

    def check_program_playing_arms(self, arms):
        sleep_time = False
        for arm in arms:
            if not rospy.get_param("/" + arm + "_sr_ur_robot_hw/headless_mode") :
                play_mode_service = rospy.ServiceProxy("/" + arm + "_sr_ur_robot_hw/dashboard/program_state", GetProgramState)
                play_msg = play_mode_service()
                if play_msg.state.state == ProgramState.STOPPED or play_msg.state.state == ProgramState.PAUSED:
                    rospy.loginfo("Not in headless mode. Starting program: %s for arm: %s", play_msg.program_name, arm)
                    self.call_dashboard_service(arm, "play")
                    sleep_time = True
            else:
                rospy.loginfo("Headless mode detected, resending robot program to arm: %s", arm)
                self.call_service(arm, "resend_robot_program")
                sleep_time = True
        return sleep_time

    def release_arm(self):
        try:
            self.check_arms_e_stops(self.arms)
            self.check_arms_protective_stop(self.arms)
            if self.check_arms_fault(self.arms):
                rospy.loginfo("Resetting robot safety, please wait approximately 15 seconds...")
                rospy.sleep(15)
            self.check_arms_protective_stop(self.arms)
            self.clear_arms_popups(self.arms)
            if self.check_arms_robot_mode(self.arms):
                self.startup_arms(self.arms)
            if self.check_program_loaded_arms(self.arms):
                rospy.sleep(2)
            if self.check_program_playing_arms(self.arms):
                rospy.sleep(5)
        except rospy.ServiceException:
            print "Service call failed for arm in: " + self.arms

if __name__ == "__main__":
    rospy.init_node("sr_ur_unlock_node")
    sr_ur_unlock = SrUrUnlock()
