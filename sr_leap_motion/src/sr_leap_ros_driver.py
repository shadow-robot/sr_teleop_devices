#!/usr/bin/python3
# Copyright 2021, 2022 Shadow Robot Company Ltd.
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


from __future__ import absolute_import, division

import Leap
import numpy
import rospy
import tf
from geometry_msgs.msg import Quaternion
from Leap import ScreenTapGesture, SwipeGesture
from leap_motion.msg import Arm, Bone, Finger, Hand, Human


class SampleListener(Leap.Listener):
    CONST_BONE_NAMES = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    CONST_FRAME_ID = "leap_hands"

    def on_init(self):
        self._human_pub = rospy.Publisher("/leap_motion/leap_device", Human, queue_size=10)
        rospy.loginfo("Initialized")

    @staticmethod
    def on_connect(controller):
        rospy.loginfo("Connected")

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)

    @staticmethod
    def on_disconnect(controller):
        rospy.loginfo("Disconnected")

    @staticmethod
    def on_exit(controller):
        rospy.loginfo("Exited")

    @staticmethod
    def quaternion_from_basis(basis, x_basis_sign):
        tf_matrix = numpy.array([[x_basis_sign * basis.x_basis.x, x_basis_sign * basis.x_basis.y,
                                  x_basis_sign * basis.x_basis.z, 0.0],
                                [basis.y_basis.x, basis.y_basis.y, basis.y_basis.z, 0.0],
                                [basis.z_basis.x, basis.z_basis.y, basis.z_basis.z, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])
        quat = tf.transformations.quaternion_from_matrix(tf_matrix)
        orientation = Quaternion()
        orientation.x = quat[0]
        orientation.y = quat[1]
        orientation.z = quat[2]
        orientation.w = quat[3]
        return orientation

    def parse_bones(self, bone, x_basis_sign):
        bone_msg = Bone()
        bone_msg.header.stamp = rospy.Time.now()
        bone_msg.header.frame_id = self.CONST_FRAME_ID
        bone_msg.type = bone.type
        bone_msg.length = bone.length/1000
        bone_msg.width = bone.width/1000
        bone_msg.bone_start.position.x = bone.prev_joint.x / 1000
        bone_msg.bone_start.position.y = bone.prev_joint.y / 1000
        bone_msg.bone_start.position.z = bone.prev_joint.z / 1000
        bone_msg.bone_end.position.x = bone.next_joint.x / 1000
        bone_msg.bone_end.position.y = bone.next_joint.y / 1000
        bone_msg.bone_end.position.z = bone.next_joint.z / 1000
        bone_msg.center.append(bone.center.x/1000)
        bone_msg.center.append(bone.center.y/1000)
        bone_msg.center.append(bone.center.z/1000)
        orientation = self.quaternion_from_basis(bone.basis, x_basis_sign)
        bone_msg.bone_end.orientation = orientation
        bone_msg.bone_start.orientation = orientation
        return bone_msg

    def parse_finger(self, finger, finger_msg, x_basis_sign):
        finger_msg.header.stamp = rospy.Time.now()
        finger_msg.header.frame_id = self.CONST_FRAME_ID
        finger_msg.lmc_finger_id = finger.id
        finger_msg.type = finger.type
        finger_msg.length = finger.length/1000
        finger_msg.width = finger.width/1000
        for bone_number in range(0, len(self.CONST_BONE_NAMES)):
            bone = finger.bone(bone_number)
            bone_msg = self.parse_bones(bone, x_basis_sign)
            finger_msg.bone_list.append(bone_msg)
        return finger_msg

    def parse_arm(self, hand, x_basis_sign):
        arm_msg = Arm()
        arm_msg.header.stamp = rospy.Time.now()
        arm_msg.header.frame_id = self.CONST_FRAME_ID
        arm_msg.elbow.position.x = hand.arm.elbow_position.x/1000
        arm_msg.elbow.position.y = hand.arm.elbow_position.y/1000
        arm_msg.elbow.position.z = hand.arm.elbow_position.z/1000
        arm_msg.center.append(hand.arm.center.x/1000)
        arm_msg.center.append(hand.arm.center.y/1000)
        arm_msg.center.append(hand.arm.center.z/1000)
        arm_msg.wrist.position.x = hand.arm.wrist_position.x / 1000
        arm_msg.wrist.position.y = hand.arm.wrist_position.y / 1000
        arm_msg.wrist.position.z = hand.arm.wrist_position.z / 1000
        arm_msg.direction.x = hand.arm.center.x/1000
        arm_msg.direction.y = hand.arm.center.y/1000
        arm_msg.direction.z = hand.arm.center.z/1000
        orientation = self.quaternion_from_basis(hand.arm.basis, x_basis_sign)
        arm_msg.elbow.orientation = orientation
        arm_msg.wrist.orientation = orientation
        return arm_msg

    def parse_hand(self, hand):
        hand_msg = Hand()
        hand_msg.header.stamp = rospy.Time.now()
        hand_msg.header.frame_id = self.CONST_FRAME_ID
        hand_msg.lmc_hand_id = hand.id
        hand_msg.is_present = True
        hand_msg.valid_gestures = False
        hand_msg.confidence = hand.confidence
        hand_msg.roll = hand.palm_normal.roll
        hand_msg.pitch = hand.direction.pitch
        hand_msg.yaw = hand.direction.yaw
        hand_msg.direction = hand.direction
        hand_msg.normal = hand.palm_normal
        hand_msg.grab_strength = hand.grab_strength
        hand_msg.pinch_strength = hand.pinch_strength
        hand_msg.time_visible = hand.time_visible
        x_basis_sign = -1.0 if hand.is_left else 1.0
        for i in range(0, 3):
            hand_msg.palm_velocity.append(hand.palm_velocity[i] / 1000)
            hand_msg.sphere_center.append(hand.sphere_center[i] / 1000)
        hand_msg.palm_center = hand.palm_position / 1000
        hand_msg.palm_width = hand.palm_width / 1000
        hand_msg.sphere_radius = hand.sphere_radius / 1000
        hand_msg.arm = self.parse_arm(hand, x_basis_sign)
        for finger in hand.fingers:
            leap_finger = Finger()
            leap_finger = self.parse_finger(finger, leap_finger, x_basis_sign)
            hand_msg.finger_list.append(leap_finger)
        return hand_msg

    def parse_human(self, frame):
        human_msg = Human()
        human_msg.header.stamp = rospy.Time.now()
        human_msg.lmc_frame_id = frame.id
        human_msg.nr_of_hands = len(frame.hands)
        human_msg.nr_of_fingers = len(frame.fingers)
        human_msg.nr_of_gestures = len(frame.gestures())
        human_msg.current_frames_per_second = frame.current_frames_per_second
        for hand in frame.hands:
            if hand.is_left:
                human_msg.left_hand = self.parse_hand(hand)
            else:
                human_msg.right_hand = self.parse_hand(hand)
        return human_msg

    def parse_frame(self, frame):
        human_msg = self.parse_human(frame)
        self._human_pub.publish(human_msg)

    def on_frame(self, controller):
        frame = controller.frame()
        # self.print_frame(frame)
        self.parse_frame(frame)

    def print_frame(self, frame):
        print("Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
               frame.id, frame.timestamp, len(frame.hands), len(frame.fingers),
               len(frame.tools), len(frame.gestures())))

    # @TODO: re-implement gesture publishing:
    # https://github.com/ros-drivers/leap_motion/blob/hydro/src/lmc_listener.cpp#L363-L399
    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"


def main():
    rospy.init_node("test")
    listener = SampleListener()
    controller = Leap.Controller()

    controller.add_listener(listener)
    rospy.spin()

    controller.remove_listener(listener)


if __name__ == "__main__":
    main()
