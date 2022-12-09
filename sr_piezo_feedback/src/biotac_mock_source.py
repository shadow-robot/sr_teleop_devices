#!/usr/bin/env python3

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

import numpy as np
import rospy
from sr_robot_msgs.msg import Biotac, BiotacAll
from std_msgs.msg import Header

if __name__ == "__main__":

    rospy.init_node("biotac_mock_source_node")
    pub = rospy.Publisher("rh/tactile", BiotacAll, queue_size=20)
    msg = BiotacAll()
    msg.header = Header()

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.get_rostime()

        for i in range(0, 5):
            biotac = Biotac()
            biotac.pac0 = 0 + int(10*np.random.rand(1))
            biotac.pac1 = 1 + int(10*np.random.rand(1))
            biotac.pac = 5 * [int(10*np.random.rand(1))]
            biotac.pdc = 2 + int(10*np.random.rand(1))
            biotac.tac = 3 + int(10*np.random.rand(1))
            biotac.tdc = 4 + int(10*np.random.rand(1))
            biotac.electrodes = 5 * [int(10*np.random.rand(1))]

            msg.tactiles[i] = biotac

        pub.publish(msg)
        rospy.Rate(120).sleep()
