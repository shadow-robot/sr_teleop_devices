#!/usr/bin/env python
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

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class CybergloveMock(object):
    def __init__(self, hand_prefix='rh'):
        self._hand_prefix = hand_prefix
        self._hand_traj_client = actionlib.SimpleActionClient('/{}_trajectory_controller'.format(self._hand_prefix) +
                                                              '/follow_joint_trajectory',
                                                              FollowJointTrajectoryAction)

    def open_hand(self):
        CONST_OPEN_HAND_JOINT_VALUES = [0] * 24

        self._send_goal_to_hand(CONST_OPEN_HAND_JOINT_VALUES)

    def pack_hand(self):
        CONST_PACK_HAND_JOINT_VALUES = [1.571, 1.57, 1.571,
                                        0.0, 1.57, 1.571,
                                        1.571, 0.0, 0.0,
                                        1.571, 1.571, 1.571,
                                        0.0, 1.571, 1.571,
                                        1.571, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.14, 0.0]

        self._send_goal_to_hand(CONST_PACK_HAND_JOINT_VALUES)

    def _send_goal_to_hand(self, hand_positions):
        self._hand_traj_client.wait_for_server()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = ["_FFJ1", "_FFJ2", "_FFJ3", "_FFJ4", "_MFJ1", "_MFJ2",
                                       "_MFJ3", "_MFJ4", "_RFJ1", "_RFJ2", "_RFJ3", "_RFJ4",
                                       "_LFJ1", "_LFJ2", "_LFJ3", "_LFJ4", "_LFJ5", "_THJ1",
                                       "_THJ2", "_THJ3", "_THJ4", "_THJ5", "_WRJ1", "_WRJ2"]
        goal.trajectory.joint_names = [self._hand_prefix + name for name in goal.trajectory.joint_names] 
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
