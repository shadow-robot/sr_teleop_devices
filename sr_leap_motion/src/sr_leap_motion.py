#!/usr/bin/python3

# Copyright 2020, 2022 Shadow Robot Company Ltd.
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
from geometry_msgs.msg import Quaternion, TransformStamped
from leap_motion.msg import Human
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class SrLeapMotion():
    HAND_LEFT = 1
    HAND_RIGHT = 2
    FINGERTIP_MODE = 1
    JOINT_ANGLE_MODE = 2
    FINGER_NAMES = ["th", "ff", "mf", "rf", "lf"]
    BONE_NAMES = ["mca", "prx", "int", "dis"]

    def __init__(self, left_root_tf, right_root_tf, bone_start=False, bone_middle=False,
                 heartbeat_topic_name="/sr_leap_motion/heartbeat"):
        self.right_root_tf_name = right_root_tf
        self.left_root_tf_name = left_root_tf
        self.bone_starts = bone_start
        self.bone_middles = bone_middle
        self.heartbeat_publisher = rospy.Publisher(heartbeat_topic_name, Bool, queue_size=1)
        self.hand = None
        self.left_hand_mode = SrLeapMotion.JOINT_ANGLE_MODE
        self.right_hand_mode = SrLeapMotion.FINGERTIP_MODE
        self.transform_broadcaster = TransformBroadcaster()
        self.left_tfs = []
        self.right_tfs = []

    def run(self):
        rospy.loginfo("Shadow leap motion running.")
        rospy.Subscriber("/leap_motion/leap_filtered", Human, self.frame)
        rospy.spin()

    def frame(self, human):
        if human.right_hand.is_present:
            if self.hand != SrLeapMotion.HAND_RIGHT:
                rospy.loginfo("Started tracking right hand!")
                self.hand = SrLeapMotion.HAND_RIGHT
        elif human.left_hand.is_present:
            if self.hand != SrLeapMotion.HAND_LEFT:
                rospy.loginfo("Started tracking left hand!")
                self.hand = SrLeapMotion.HAND_LEFT
        else:
            if self.hand is not None:
                rospy.loginfo("Lost sight of any hands.")
                self.hand = None
        self.left_tfs = self.common_frame(human=human, hand=human.left_hand, arm_prefix="la_", hand_prefix="lh_")
        self.right_tfs = self.common_frame(human=human, hand=human.right_hand, arm_prefix="ra_", hand_prefix="rh_")
        if self.left_tfs:
            self.publish_tfs(transforms=self.left_tfs, root_tf_name=self.left_root_tf_name)
        if self.right_tfs:
            self.publish_tfs(transforms=self.right_tfs, root_tf_name=self.right_root_tf_name)
        self.heartbeat_publisher.publish(not ((not self.left_tfs) and (not self.right_tfs)))

    def publish_tfs(self, transforms, root_tf_name):
        tf_buffer = Buffer()
        tf_authority = ""
        root_tf_found = False
        for transform in transforms:
            tf_buffer.set_transform(transform, tf_authority)
            if transform.child_frame_id == root_tf_name:
                root_tf_found = True
        # If the requested root TF name isn't one of the TFs reported by Leap Motion, report all TFs relative to Leap
        # sensor, named as the requested root TF name
        if not root_tf_found:
            for transform in transforms:
                transform.header.frame_id = root_tf_name
                self.transform_broadcaster.sendTransform(transform)
        # If the requested root TF name is one of the TFs reported by Leap Motion, report all TFs relative to that TF
        else:
            for transform in transforms:
                if transform.child_frame_id != root_tf_name:
                    reparented_tf = tf_buffer.lookup_transform(root_tf_name, transform.child_frame_id, rospy.Time())
                    self.transform_broadcaster.sendTransform(reparented_tf)

    def common_frame(self, human, hand=None, arm_prefix="ra_", hand_prefix="rh_"):
        new_tfs = []
        if hand is None:
            hand = human.right_hand
        if not hand.is_present:
            return new_tfs
        elbow_tf = self.new_root_tf(f"{arm_prefix}leap_elbow", human.header.stamp)
        self.leap_pose_to_ros_tf(hand.arm.elbow, elbow_tf.transform)
        palm_tf = self.new_root_tf(f"{hand_prefix}leap_palm", human.header.stamp)
        palm_tf.transform.translation.x = -hand.palm_center.z
        palm_tf.transform.translation.y = -hand.palm_center.x
        palm_tf.transform.translation.z = hand.palm_center.y
        palm_tf.transform.rotation = self.ros_rpy_to_quat([-hand.roll, -hand.pitch, -hand.yaw])
        wrist_tf = self.new_root_tf(f"{arm_prefix}leap_wrist", human.header.stamp)
        self.leap_pose_to_ros_tf(hand.arm.wrist, wrist_tf.transform)
        arm_tf = self.new_root_tf(f"{arm_prefix}leap_arm_center", human.header.stamp)
        arm_tf.transform.translation.x = -hand.arm.center[2]
        arm_tf.transform.translation.y = -hand.arm.center[0]
        arm_tf.transform.translation.z = hand.arm.center[1]
        arm_tf.transform.rotation = elbow_tf.transform.rotation
        new_tfs.append(elbow_tf)
        new_tfs.append(wrist_tf)
        new_tfs.append(palm_tf)
        new_tfs.append(arm_tf)
        for finger in hand.finger_list:
            for bone in finger.bone_list:
                bone_end_tf = self.new_root_tf(f"{hand_prefix}leap_{SrLeapMotion.FINGER_NAMES[finger.type]}"
                                               f"_{SrLeapMotion.BONE_NAMES[bone.type]}_end",
                                               human.header.stamp)
                self.leap_pose_to_ros_tf(bone.bone_end, bone_end_tf.transform)
                new_tfs.append(bone_end_tf)
                if self.bone_starts:
                    bone_start_tf = self.new_root_tf(f"{hand_prefix}leap_{SrLeapMotion.FINGER_NAMES[finger.type]}"
                                                     f"_{SrLeapMotion.BONE_NAMES[bone.type]}_start",
                                                     human.header.stamp)
                    self.leap_pose_to_ros_tf(bone.bone_start, bone_start_tf.transform)
                    new_tfs.append(bone_start_tf)
                if self.bone_middles:
                    bone_mid_tf = self.new_root_tf(f"{hand_prefix}leap_{SrLeapMotion.FINGER_NAMES[finger.type]}"
                                                   f"_{SrLeapMotion.BONE_NAMES[bone.type]}_mid",
                                                   human.header.stamp)
                    bone_mid_tf.transform.translation.x = -bone.center[2]
                    bone_mid_tf.transform.translation.y = -bone.center[0]
                    bone_mid_tf.transform.translation.z = bone.center[1]
                    bone_mid_tf.transform.rotation = bone_end_tf.transform.rotation
                    new_tfs.append(bone_mid_tf)
        return new_tfs

    @staticmethod
    def new_root_tf(name, timestamp):
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = "root"
        transform.child_frame_id = name
        transform.transform.rotation.w = 1.0
        return transform

    def leap_pose_to_ros_tf(self, leap_pose, ros_tf):
        self.leap_orientation_to_ros_rotation(leap_pose.orientation, ros_tf.rotation)
        self.leap_position_to_ros_translation(leap_pose.position, ros_tf.translation)

    @staticmethod
    def leap_orientation_to_ros_rotation(leap_orientation, ros_rotation):
        # Ros roll is leap -yaw
        # Ros pitch is leap -roll
        # Ros yaw is leap pitch
        leap_rpy = euler_from_quaternion([leap_orientation.x, leap_orientation.y, leap_orientation.z,
                                          leap_orientation.w])
        ros_quaternion = quaternion_from_euler(leap_rpy[2], leap_rpy[0], -leap_rpy[1])
        ros_rotation.x = ros_quaternion[0]
        ros_rotation.y = ros_quaternion[1]
        ros_rotation.z = ros_quaternion[2]
        ros_rotation.w = ros_quaternion[3]

    @staticmethod
    def leap_position_to_ros_translation(leap_position, ros_translation):
        # Ros x is leap -z
        # Ros y is leap -x
        # Ros z is leap y
        ros_translation.x = -leap_position.z
        ros_translation.y = -leap_position.x
        ros_translation.z = leap_position.y

    @staticmethod
    def ros_rpy_to_quat(ros_rpy):
        quat_array = quaternion_from_euler(ros_rpy[0], ros_rpy[1], ros_rpy[2])
        quat = Quaternion()
        quat.x = quat_array[0]
        quat.y = quat_array[1]
        quat.z = quat_array[2]
        quat.w = quat_array[3]
        return quat


if __name__ == "__main__":
    rospy.init_node("sr_leap_motion")
    left_root_tf_name = rospy.get_param("~left_root_tf_name", "sr_leap_motion_root")
    right_root_tf_name = rospy.get_param("~right_root_tf_name", "sr_leap_motion_root")
    bone_starts = rospy.get_param("~bone_starts", False)
    bone_middles = rospy.get_param("~bone_middles", False)
    heartbeat_topic = rospy.get_param("~heartbeat_topic", "/sr_leap_motion/heartbeat")
    sr_leap_motion = SrLeapMotion(left_root_tf=left_root_tf_name, right_root_tf=right_root_tf_name,
                                  bone_start=bone_starts, bone_middle=bone_middles,
                                  heartbeat_topic_name=heartbeat_topic)
    sr_leap_motion.run()
