#!/usr/bin/python3

from __future__ import absolute_import
import Leap
import sys, threading, time
import rospy
from leap_motion.msg import Human, Hand, Finger, Bone, Arm
import std_msgs.msg
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture


class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']
    CONST_FRAME_ID = "leap_hands"

    def on_init(self, controller):
        self._leap_ros_publisher = rospy.Publisher("/leap_motion/human_out", Hand, queue_size=10)
        self._finger_pub = rospy.Publisher("/leap_motion/finger", Finger, queue_size=10)
        self._human_pub = rospy.Publisher("/leap_motion/human", Human, queue_size=10)
        print("Initialized")

    def on_connect(self, controller):
        print("Connected")

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print("Disconnected")

    def on_exit(self, controller):
        print("Exited")
        
    def parse_bones(self, bone, is_left):
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
        # @TODO - get bone rotation using basis vectors for left hand 
        return bone_msg
    
    def parse_finger(self, finger, finger_msg, is_left):
        finger_msg.header.stamp = rospy.Time.now()
        finger_msg.header.frame_id = self.CONST_FRAME_ID
        finger_msg.lmc_finger_id = finger.id
        finger_msg.type = finger.type
        finger_msg.length = finger.length/1000
        finger_msg.width = finger.width/1000
        for b in range(0, 4):
            bone = finger.bone(b)
            bone_msg = self.parse_bones(bone, is_left)
            finger_msg.bone_list.append(bone_msg)
        return finger_msg

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
        for i in range(0, 3):
            hand_msg.palm_velocity.append(hand.palm_velocity[i] / 1000)
            hand_msg.sphere_center.append(hand.sphere_center[i] / 1000)
        hand_msg.palm_center = hand.palm_position / 1000
        hand_msg.palm_width = hand.palm_width / 1000
        hand_msg.sphere_radius = hand.sphere_radius / 1000
        # hand_msg.to_string
        for finger in hand.fingers:
            f = Finger()
            f = self.parse_finger(finger, f, hand.is_left)
            hand_msg.finger_list.append(f)
        return hand_msg
            
    def parse_human(self, frame):
        human_msg = Human()
        human_msg.header.stamp = rospy.Time.now()
        human_msg.lmc_frame_id = frame.id
        human_msg.nr_of_hands = len(frame.hands)
        human_msg.nr_of_fingers = len(frame.fingers)
        human_msg.nr_of_gestures = len(frame.gestures())
        human_msg.current_frames_per_second = frame.current_frames_per_second
        # human_msg.to_string
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
        # Get the most recent frame and report some basic information
        frame = controller.frame()
        self.parse_frame(frame)
        print("Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures())))

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
