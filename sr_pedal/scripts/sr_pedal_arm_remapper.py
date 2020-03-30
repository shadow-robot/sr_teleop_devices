#!/usr/bin/python

import rospy
from std_msgs.msg import Bool
from sr_pedal.msg import Status
    
class SrPedalArmRemapper():
    def __init__(self):
        self.subscriber = rospy.Subscriber("sr_pedal/status", Status, self.pedal_sub)
        self.publisher = rospy.Publisher("sr_arm/release", Bool, queue_size=1)

    def pedal_sub(self, data):
        if data.connected and data.left_pressed and data.right_pressed:
            self.publisher.publish(True)

if __name__ == '__main__':
    rospy.init_node('sr_pedal_arm_remapper', anonymous=True)
    sr_pedal_arm_remapper = SrPedalArmRemapper()
    rospy.spin()
