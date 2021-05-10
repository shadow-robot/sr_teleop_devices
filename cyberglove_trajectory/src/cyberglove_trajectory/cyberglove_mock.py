#!/usr/bin/env python3
#
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

from __future__ import absolute_import
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class CybergloveMock(object):
    def __init__(self, hand_prefix='rh', wrist_control=True, hand_type='hand_e'):
        self._hand_prefix = hand_prefix
        self._wrist_control = wrist_control
        self.hand_type = hand_type
        self._set_traj_server('/{}_trajectory_controller/follow_joint_trajectory'.format(self._hand_prefix))
        self.joint_names = []

        self._set_up_joint_list()

    def _set_traj_server(self, traj_server_ns):
        self._hand_traj_ns = traj_server_ns
        self._hand_traj_client = actionlib.SimpleActionClient(self._hand_traj_ns,
                                                              FollowJointTrajectoryAction)

    def _set_up_joint_list(self):
        if 'hand_e' == self.hand_type:
            self.joint_names = ["_FFJ1", "_FFJ2", "_FFJ3", "_FFJ4", "_MFJ1", "_MFJ2",
                                "_MFJ3", "_MFJ4", "_RFJ1", "_RFJ2", "_RFJ3", "_RFJ4",
                                "_LFJ1", "_LFJ2", "_LFJ3", "_LFJ4", "_LFJ5", "_THJ1",
                                "_THJ2", "_THJ3", "_THJ4", "_THJ5"]
            if self._wrist_control:
                self.joint_names += ["_WRJ1", "_WRJ2"]
        else:
            raise ValueError("Unsupported hand type!")

        self.joint_names = [self._hand_prefix + name for name in self.joint_names]

    def open_hand(self):
        num_of_joints = len(self.joint_names)
        CONST_OPEN_HAND_JOINT_VALUES = [0] * num_of_joints
        self._send_goal_to_hand(CONST_OPEN_HAND_JOINT_VALUES)

    def pack_hand(self):
        if 'hand_e' == self.hand_type:
            CONST_PACK_HAND_JOINT_VALUES = [1.571, 1.571, 1.571, 0.0,
                                            1.571, 1.571, 1.571, 0.0,
                                            1.571, 1.571, 1.571, 0.0,
                                            1.571, 1.571, 1.571, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0]
            if self._wrist_control:
                CONST_PACK_HAND_JOINT_VALUES += [0.14, 0.0]
        else:
            raise ValueError("Unsupported hand type!")

        self._send_goal_to_hand(CONST_PACK_HAND_JOINT_VALUES)

    def _send_goal_to_hand(self, hand_positions):
        if not self._hand_traj_client.wait_for_server(timeout=rospy.Duration(3)):
            rospy.logwarn("Failed to connected to actionlib server '{}'.".format(self._hand_traj_ns))
            return
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = self.joint_names
        joint_traj_point = JointTrajectoryPoint()

        joint_traj_point.positions = hand_positions

        joint_traj_point.time_from_start = rospy.Duration(0.1)
        goal.trajectory.points.append(joint_traj_point)
        self._hand_traj_client.send_goal(goal)

    def run(self):
        rospy.loginfo("Alternating between pack and open every 3 seconds...")
        while not rospy.is_shutdown():
            self.pack_hand()
            rospy.sleep(3)
            self.open_hand()
            rospy.sleep(3)


if __name__ == "__main__":
    rospy.init_node('cyberglove_mock_node')

    cyberglove_mock = CybergloveMock()
    cyberglove_mock.run()
