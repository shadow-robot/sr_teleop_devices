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
from ur_dashboard_msgs.srv import GetSafetyMode, GetProgramState, GetRobotMode
from ur_dashboard_msgs.msg import SafetyMode, ProgramState, RobotMode
from std_srvs.srv import Trigger

class SrUrUnlock():
    def __init__(self):
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

    def release_arm(self):
        # Namespace will change with bimanual
        rospy.wait_for_service("/ur_hardware_interface/dashboard/get_safety_mode")
        try:
            safety_mode_service = rospy.ServiceProxy("/ur_hardware_interface/dashboard/get_safety_mode", GetSafetyMode)
            safety_mode_msg = safety_mode_service()
            if safety_mode_msg.safety_mode.mode == SafetyMode.PROTECTIVE_STOP:
                serv_call = rospy.ServiceProxy("/ur_hardware_interface/dashboard/unlock_protective_stop", Trigger)
                resp = serv_call()
            if safety_mode_msg.safety_mode.mode == SafetyMode.FAULT:
                serv_call = rospy.ServiceProxy("/ur_hardware_interface/dashboard/restart_safety", Trigger)
                resp = serv_call()
            serv_call = rospy.ServiceProxy("/ur_hardware_interface/dashboard/close_safety_popup", Trigger)
            resp = serv_call()
            rospy.sleep(1)
            serv_call = rospy.ServiceProxy("/ur_hardware_interface/dashboard/close_popup", Trigger)
            resp = serv_call()
            rospy.sleep(1)
            get_mode_service = rospy.ServiceProxy("/ur_hardware_interface/dashboard/get_robot_mode", GetRobotMode)
            get_mode_msg = get_mode_service()
            if get_mode_msg.robot_mode.mode == RobotMode.IDLE or get_mode_msg.robot_mode.mode == RobotMode.POWER_OFF:
                serv_call = rospy.ServiceProxy("/ur_hardware_interface/dashboard/brake_release", Trigger)
                resp = serv_call()
                rospy.sleep(1)
            play_mode_service = rospy.ServiceProxy("/ur_hardware_interface/dashboard/program_state", GetProgramState)
            play_msg = play_mode_service()
            if play_msg.state.state == ProgramState.STOPPED or play_msg.state.state == ProgramState.PAUSED:                
                rospy.sleep(5)
                serv_call = rospy.ServiceProxy("/ur_hardware_interface/dashboard/play", Trigger)
                resp = serv_call()
        except rospy.ServiceException:
            print("Service call failed")

if __name__ == "__main__":
    rospy.init_node("sr_ur_unlock_node")
    sr_ur_unlock = SrUrUnlock()
