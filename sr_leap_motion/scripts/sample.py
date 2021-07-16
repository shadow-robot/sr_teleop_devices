#!/usr/bin/python3
################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################
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

    def on_init(self, controller):
        self._leap_ros_publisher = rospy.Publisher("/leap_motion/human_out", Hand, queue_size=10)
        self._finger_pub = rospy.Publisher("/leap_motion/finger", Finger, queue_size=10)
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
        
    def parse_bones(self, bone):
        bone_msg = Bone()
        bone_msg.header.stamp = rospy.Time.now()
        bone_msg.type = bone.type
        return bone_msg
    
    def parse_finger(self, finger):
        finger_msg = Finger()
        finger_msg.header = std_msgs.msg.Header()
        finger_msg.header.stamp = rospy.Time.now()
        #finger_header = std_msgs.msg.Header()
        #finger_header.stamp = rospy.Time.now()
        #finger_msg.header = finger_header
        finger_msg.lmc_finger_id = finger.id
        finger_msg.type = finger.type
        finger_msg.length = finger.length/1000
        finger_msg.width = finger.width/1000
        for bone in [finger.bone(b) for b in range(0, 4)]:
            finger_msg.bone_list.append(self.parse_bones(bone))
            

    def parse_hand(self, hand):
        hand_msg = Hand()
        hand_msg.header.stamp = rospy.Time.now()
        hand_msg.lmc_hand_id = hand.id
        hand_msg.roll = hand.palm_normal.roll
        hand_msg.pitch = hand.direction.pitch
        hand_msg.yaw = hand.direction.yaw
        hand_msg.direction = hand.direction
        hand_msg.normal = hand.palm_normal
        hand_msg.is_present = True
        for finger in hand.fingers:
            hand_msg.finger_list.append(self.parse_finger(finger))
        return hand_msg
            
    def parse_human(self, frame):
        human_msg = Human()
        #header = std_msgs.msg.Header()
        #header.stamp = rospy.Time().now()
        #print(header.stamp.secs)
        #print(type(header.stamp.secs))
        human_msg.header.stamp = rospy.Time.now()
        human_msg.lmc_frame_id = frame.id
        for hand in frame.hands:
            if hand.is_left:
                human_msg.left_hand = self.parse_hand(hand)
            else:
                human_msg.right_hand = self.parse_hand(hand)
        return human_msg
            
        
    def parse_frame(self, frame):
        hand = frame.hands[0]
        human = self.parse_human(frame)
        hand_msg = self.parse_hand(frame.hands[0])
        hand_msg = Hand()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        hand_msg.header = header
        hand_msg.lmc_hand_id = hand.id
        hand_msg.roll = hand.palm_normal.roll
        hand_msg.pitch = hand.direction.pitch
        hand_msg.yaw = hand.direction.yaw
        hand_msg.direction = hand.direction
        hand_msg.normal = hand.palm_normal
        finger_msg = Finger()
        for finger in hand.fingers:
            #hand_msg.finger_list.append(self.parse_finger(finger))
            f = Finger()
            f.header.stamp = rospy.Time.now()
            f.lmc_finger_id = finger.id
            f.type = finger.type
            f.length = finger.length/1000
            f.width = finger.width/1000
            hand_msg.finger_list.append(f)
            for bone in [finger.bone(b) for b in range(0, 4)]:
                b = Bone()
                b.header.stamp = rospy.Time.now()
                b.type = bone.type
                f.bone_list.append(b)
        self._leap_ros_publisher.publish(hand_msg)
        return
        bones = self.parse_bones(frame)
        fingers = self.parse_finger(frame, bones)
        hands = self.parse_hand(frame, fingers)
        human = self.parse_human(frame, hands)
        #header = std_msgs.msg.Header()
        #header.stamp = rospy.Time.now()
        human_ros_msg = Human()
        #human_ros_msg.header = header
        human_ros_msg.lmc_frame_id = frame.id
        for hand in frame.hands:
            hand_ros_msg = Hand()
            hand_ros_msg.lmc_hand_id = hand.id
            #hand_ros_msg.is_present
            #hand_ros_msg.confidence
            hand_ros_msg.roll = hand.palm_normal.roll
            hand_ros_msg.pitch = hand.direction.pitch
            hand_ros_msg.yaw = hand.direction.yaw
            hand_ros_msg.direction = hand.direction
            hand_ros_msg.normal = hand.palm_normal
            for finger in hand.fingers:
                finger_ros_msg = Finger()
                finger_header = std_msgs.msg.Header()
                finger_ros_msg.header = finger_header
                finger_ros_msg.lmc_finger_id = finger.id
                finger_ros_msg.type = finger.type
                finger_ros_msg.length = finger.length/1000
                finger_ros_msg.width = finger.width/1000
                for bone in [finger.bone(b) for b in range(0, 4)]:
                    bone_ros_msg = Bone()
                    bone_ros_msg.type = bone.type
                    bone_ros_msg.to_string = self.bone_names[bone.type]
                    finger_ros_msg.bone_list.append(bone_ros_msg)
                hand_ros_msg.finger_list.append(finger)
            arm = Arm()
            arm.elbow.position = hand.arm.elbow_position
            arm.wrist.position = hand.arm.wrist_position
            arm.direction.x = hand.arm.direction.x
            arm.direction.y = hand.arm.direction.y
            arm.direction.z = hand.arm.direction.z
            # print(type(hand.arm.wrist_position))
            hand_ros_msg.arm = arm
            if hand.is_left:
                human_ros_msg.left_hand = hand_ros_msg
            else:
                human_ros_msg.right_hand = hand_ros_msg        
        self._leap_ros_publisher.publish(human_ros_msg)
        

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()
        self.parse_frame(frame)

        print("Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures())))
              
        
    '''
        # Get hands
        #human_ros_msg = Human()
        

            handType = "Left hand" if hand.is_left else "Right hand"

            print("  %s, id %d, position: %s" % (
                handType, hand.id, hand.palm_position))

            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction

            # Calculate the hand's pitch, roll, and yaw angles
            print("  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                direction.pitch * Leap.RAD_TO_DEG,
                normal.roll * Leap.RAD_TO_DEG,
                direction.yaw * Leap.RAD_TO_DEG))

            # Get arm bone
            arm = hand.arm
            print("  Arm direction: %s, wrist position: %s, elbow position: %s" % (
                arm.direction,
                arm.wrist_position,
                arm.elbow_position))

            # Get fingers
            for finger in hand.fingers:

                print("    %s finger, id: %d, length: %fmm, width: %fmm" % (
                    self.finger_names[finger.type],
                    finger.id,
                    finger.length,
                    finger.width))

                # Get bones
                for b in range(0, 4):
                    bone = finger.bone(b)
                    print("      Bone: %s, start: %s, end: %s, direction: %s" % (
                        self.bone_names[bone.type],
                        bone.prev_joint,
                        bone.next_joint,
                        bone.direction))

        # Get tools
        for tool in frame.tools:

            print("  Tool id: %d, position: %s, direction: %s" % (
                tool.id, tool.tip_position, tool.direction))

        # Get gestures
        for gesture in frame.gestures():
            if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                circle = CircleGesture(gesture)

                # Determine clock direction using the angle between the pointable and the circle normal
                if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/2:
                    clockwiseness = "clockwise"
                else:
                    clockwiseness = "counterclockwise"

                # Calculate the angle swept since the last frame
                swept_angle = 0
                if circle.state != Leap.Gesture.STATE_START:
                    previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
                    swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

                print("  Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
                        gesture.id, self.state_names[gesture.state],
                        circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness))

            if gesture.type == Leap.Gesture.TYPE_SWIPE:
                swipe = SwipeGesture(gesture)
                print("  Swipe id: %d, state: %s, position: %s, direction: %s, speed: %f" % (
                        gesture.id, self.state_names[gesture.state],
                        swipe.position, swipe.direction, swipe.speed))

            if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
                keytap = KeyTapGesture(gesture)
                print("  Key Tap id: %d, %s, position: %s, direction: %s" % (
                        gesture.id, self.state_names[gesture.state],
                        keytap.position, keytap.direction ))

            if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
                screentap = ScreenTapGesture(gesture)
                print("  Screen Tap id: %d, %s, position: %s, direction: %s" % (
                        gesture.id, self.state_names[gesture.state],
                        screentap.position, screentap.direction ))

        if not (frame.hands.is_empty and frame.gestures().is_empty):
            print("")
    '''        

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
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)
    rospy.spin()
    # Keep this process running until Enter is pressed
    print("Press Enter to quit...")
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
