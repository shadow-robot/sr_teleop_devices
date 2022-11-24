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

from __future__ import absolute_import

import rospy
from geometry_msgs.msg import Quaternion, TransformStamped
from leap_motion.msg import Human
from sr_leap_motion.srv import Skeleton, SkeletonResponse
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, LookupException
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
        self.bone_starts = bone_start
        self.bone_middles = bone_middle
        self.heartbeat_publisher = rospy.Publisher(heartbeat_topic_name, Bool, queue_size=1)
        self.hands = []
        self.transform_broadcaster = TransformBroadcaster()
        self.tf_structure = self.default_tf_structure()
        self.set_root_tf("lh", left_root_tf)
        self.set_root_tf("rh", right_root_tf)
        self.skeleton_service = rospy.Service('/sr_leap_motion/skeleton', Skeleton, self.on_skeleton_request)
        self.local_tf_buffer = Buffer()
        self.human = None

    def on_skeleton_request(self, request):
        response = SkeletonResponse()
        names = "["
        translations = ""
        root_transform = self.local_tf_buffer.lookup_transform(f'{request.root_tf_name}', f'rh_leap_mf_mca_end', rospy.Time.now(), rospy.Duration(1.0))
        time = root_transform.header.stamp
        rotations = ""
        rotation_fix_tf = TransformStamped()
        rotation_fix_tf.header.stamp = time
        rotation_fix_tf.transform.rotation.x = 0.5
        rotation_fix_tf.transform.rotation.y = -0.5
        rotation_fix_tf.transform.rotation.z = 0.5
        rotation_fix_tf.transform.rotation.w = 0.5
        rotation_fix_tf.header.frame_id = f'{request.root_tf_name}'
        rotation_fix_tf.child_frame_id = f'{request.root_tf_name}_manus'
        self.local_tf_buffer.set_transform(rotation_fix_tf, "local_buffer")
        tf_names = [f'rh_leap_th_prx_start', f'rh_leap_th_prx_end', f'rh_leap_th_int_end', f'rh_leap_th_dis_end']
        for finger_name in ["ff", "mf", "rf", "lf"]:
            tf_names.extend([f'rh_leap_{finger_name}_mca_start', f'rh_leap_{finger_name}_mca_end', f'rh_leap_{finger_name}_prx_end', f'rh_leap_{finger_name}_int_end', f'rh_leap_{finger_name}_dis_end'])
        if (request.root_tf_name != ""):
            for tf_name in tf_names:
                try:
                    rotation_fix_tf.header.frame_id = f'{tf_name}'
                    rotation_fix_tf.child_frame_id = f'{tf_name}_manus'
                    self.local_tf_buffer.set_transform(rotation_fix_tf, "local_buffer")
                    transform = self.local_tf_buffer.lookup_transform(f'{request.root_tf_name}_manus', f'{tf_name}_manus', time)
                    names = f'{names}, "{tf_name}"'
                    response.transforms.append(transform)
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    translations = f'{translations}{{{round(translation.x, 5)}f, {round(translation.y, 5)}f, {round(translation.z, 5)}f}}, '
                    rotations = f'{rotations}{{{round(rotation.x, 5)}f, {round(rotation.y, 5)}f, {round(rotation.z, 5)}f, {round(rotation.w, 5)}f}}, '
                    response.manus_skeleton = f'{response.manus_skeleton}{translation.x}f, {translation.y}f, {translation.z}f, {tf_name}\n'
                except LookupException as e:
                    raise rospy.ServiceException(f'Failed to find skeleton transform: {e}')
        response.human = self.human
        rospy.loginfo(response.manus_skeleton)
        rospy.loginfo(f'{names}]')
        rospy.loginfo(f'{translations}')
        rospy.loginfo(f'{rotations}')
        return response

    def run(self):
        rospy.loginfo("Shadow leap motion running.")
        rospy.Subscriber("/leap_motion/leap_filtered", Human, self.frame)
        rospy.spin()

    def frame(self, human):
        self.human = human
        all_tfs = []
        if self.human.right_hand.is_present:
            if not SrLeapMotion.HAND_RIGHT in self.hands:
                rospy.loginfo("Started tracking right hand!")
                self.hands.append(SrLeapMotion.HAND_RIGHT)
            all_tfs = all_tfs + self.common_frame(hand=self.human.right_hand, hand_prefix="rh")
        else:
            if SrLeapMotion.HAND_RIGHT in self.hands:
                rospy.loginfo("Stopped tracking right hand!")
                self.hands.remove(SrLeapMotion.HAND_RIGHT)
        if self.human.left_hand.is_present:
            if not SrLeapMotion.HAND_LEFT in self.hands:
                rospy.loginfo("Started tracking left hand!")
                self.hands.append(SrLeapMotion.HAND_LEFT)
            all_tfs = all_tfs + self.common_frame(hand=self.human.left_hand, hand_prefix="lh")
        else:
            if SrLeapMotion.HAND_LEFT in self.hands:
                rospy.loginfo("Stopped tracking left hand!")
                self.hands.remove(SrLeapMotion.HAND_LEFT)
        # left_tfs = self.common_frame(hand=self.human.left_hand, hand_prefix="lh")
        # right_tfs = self.common_frame(hand=self.human.right_hand, hand_prefix="rh")
        # all_tfs = left_tfs + right_tfs
        all_tfs = self.reparent_tfs(all_tfs)
        self.transform_broadcaster.sendTransform(all_tfs)
        self.heartbeat_publisher.publish(all_tfs)

    def reparent_tfs(self, transforms):
        new_tfs = []
        right_present = False
        left_present = False
        for transform in transforms:
            self.local_tf_buffer.set_transform(transform, "local_buffer")
            right_present = right_present or transform.header.frame_id == "rh_leap_camera"
            left_present = left_present or transform.header.frame_id == "lh_leap_camera"
        for transform in transforms:
            if transform.child_frame_id in self.tf_structure:
                new_tfs.append(self.local_tf_buffer.lookup_transform(self.tf_structure[transform.child_frame_id], transform.child_frame_id, transform.header.stamp))
        if right_present and self.right_root_tf_name != "rh_leap_camera":
            new_tfs.append(self.local_tf_buffer.lookup_transform(self.right_root_tf_name, "rh_leap_camera", transform.header.stamp))
        if left_present and self.left_root_tf_name != "lh_leap_camera":
            new_tfs.append(self.local_tf_buffer.lookup_transform(self.left_root_tf_name, "lh_leap_camera", transform.header.stamp))
        return new_tfs

    def publish_tfs(self, transforms):
        self.transform_broadcaster.sendTransform(transforms)

    def common_frame(self, hand, hand_prefix="rh"):
        new_tfs = []
        if not hand.is_present:
            return new_tfs
        elbow_tf = self.new_root_tf(f'{hand_prefix}_leap_elbow', hand.header.stamp, hand_prefix)
        self.leap_pose_to_ros_tf(hand.arm.elbow, elbow_tf.transform)
        palm_tf = self.new_root_tf(f'{hand_prefix}_leap_palm', hand.header.stamp, hand_prefix)
        palm_tf.transform.translation.x = -hand.palm_center.z
        palm_tf.transform.translation.y = -hand.palm_center.x
        palm_tf.transform.translation.z = hand.palm_center.y
        palm_tf.transform.rotation = self.ros_rpy_to_quat([-hand.roll, -hand.pitch, -hand.yaw])
        wrist_tf = self.new_root_tf(f'{hand_prefix}_leap_wrist', hand.header.stamp, hand_prefix)
        self.leap_pose_to_ros_tf(hand.arm.wrist, wrist_tf.transform)
        arm_tf = self.new_root_tf(f'{hand_prefix}_leap_arm_center', hand.header.stamp, hand_prefix)
        arm_tf.transform.translation.x = -hand.arm.center[2]
        arm_tf.transform.translation.y = -hand.arm.center[0]
        arm_tf.transform.translation.z = hand.arm.center[1]
        arm_tf.transform.rotation = elbow_tf.transform.rotation
        new_tfs.append(elbow_tf)
        new_tfs.append(wrist_tf)
        new_tfs.append(palm_tf)
        new_tfs.append(arm_tf)
        for finger in hand.finger_list:
            finger_name = SrLeapMotion.FINGER_NAMES[finger.type]
            for bone in finger.bone_list:
                bone_name = SrLeapMotion.BONE_NAMES[bone.type]
                bone_end_tf = self.new_root_tf(
                    f'{hand_prefix}_leap_{finger_name}_{bone_name}_end',
                    hand.header.stamp, hand_prefix)
                self.leap_pose_to_ros_tf(bone.bone_end, bone_end_tf.transform)
                new_tfs.append(bone_end_tf)
                if self.bone_starts:
                    bone_start_tf = self.new_root_tf(
                        f'{hand_prefix}_leap_{finger_name}_{bone_name}_start',
                        hand.header.stamp, hand_prefix)
                    self.leap_pose_to_ros_tf(bone.bone_start, bone_start_tf.transform)
                    new_tfs.append(bone_start_tf)
                if self.bone_middles:
                    bone_mid_tf = self.new_root_tf(
                        f'{hand_prefix}_leap_{finger_name}_{bone_name}_mid',
                        hand.header.stamp, hand_prefix)
                    bone_mid_tf.transform.translation.x = -bone.center[2]
                    bone_mid_tf.transform.translation.y = -bone.center[0]
                    bone_mid_tf.transform.translation.z = bone.center[1]
                    bone_mid_tf.transform.rotation = bone_end_tf.transform.rotation
                    new_tfs.append(bone_mid_tf)
        return new_tfs

    @staticmethod
    def new_root_tf(name, timestamp, side_prefix="rh"):
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = f'{side_prefix}_leap_camera'
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

    def set_root_tf(self, side="lh", root_tf_name="lh_leap_camera"):
        if side == "rh":
            self.right_root_tf_name = root_tf_name
        if side == "lh":
            self.left_root_tf_name = root_tf_name
        self.tf_structure.update({f'{side}_leap_camera': root_tf_name})
        if root_tf_name in self.tf_structure:
            del self.tf_structure[root_tf_name]
        # rospy.loginfo(self.tf_structure)

    def default_tf_structure(self):
        tf_structure = {}
        for side in ["rh", "lh"]:
            tf_structure.update({
                f'{side}_leap_elbow': f'{side}_leap_camera', f'{side}_leap_arm_center': f'{side}_leap_elbow',
                f'{side}_leap_wrist': f'{side}_leap_arm_center', f'{side}_leap_palm': f'{side}_leap_wrist'})
            for finger in ["ff", "mf", "rf", "lf", "th"]:
                tf_structure.update(self.default_bone_tf_structure(side=side, bone="mca", finger=finger, default_root="wrist"))
                tf_structure.update(self.default_bone_tf_structure(side=side, bone="prx", finger=finger, default_root=f'{finger}_mca_end'))
                tf_structure.update(self.default_bone_tf_structure(side=side, bone="int", finger=finger, default_root=f'{finger}_prx_end'))
                tf_structure.update(self.default_bone_tf_structure(side=side, bone="dis", finger=finger, default_root=f'{finger}_int_end'))
        return tf_structure      

    def default_bone_tf_structure(self, side="rh", bone="mca", finger="ff", default_root="wrist"):
        tf_structure = {}
        tf_structure.update({f'{side}_leap_{finger}_{bone}_end': f'{side}_leap_{default_root}'})
        if (self.bone_starts):
                tf_structure.update({f'{side}_leap_{finger}_{bone}_start': f'{side}_leap_{default_root}'})
                tf_structure.update({f'{side}_leap_{finger}_{bone}_end': f'{side}_leap_{finger}_{bone}_start'})
        if (self.bone_middles):
            tf_structure.update({f'{side}_leap_{finger}_{bone}_mid': f'{side}_leap_{default_root}'})
            tf_structure.update({f'{side}_leap_{finger}_{bone}_end': f'{side}_leap_{finger}_{bone}_mid'})
            if (self.bone_starts):
                tf_structure.update({f'{side}_leap_{finger}_{bone}_mid': f'{side}_leap_{finger}_{bone}_start'})
        return tf_structure


if __name__ == "__main__":
    rospy.init_node("sr_leap_motion")
    left_root_tf_name = rospy.get_param("~left_root_tf_name", "lh_leap_camera")
    right_root_tf_name = rospy.get_param("~right_root_tf_name", "rh_leap_camera")
    bone_starts = rospy.get_param("~bone_starts", False)
    bone_middles = rospy.get_param("~bone_middles", False)
    heartbeat_topic = rospy.get_param("~heartbeat_topic", "/sr_leap_motion/heartbeat")
    sr_leap_motion = SrLeapMotion(left_root_tf=left_root_tf_name, right_root_tf=right_root_tf_name,
                                  bone_start=bone_starts, bone_middle=bone_middles,
                                  heartbeat_topic_name=heartbeat_topic)
    sr_leap_motion.run()
