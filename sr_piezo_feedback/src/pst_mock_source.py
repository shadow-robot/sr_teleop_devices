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

from __future__ import absolute_import

import numpy as np
import rospy
from sr_robot_msgs.msg import ShadowPST

if __name__ == "__main__":

    rospy.init_node("pst_mock_source_node")
    pub = rospy.Publisher("rh/tactile", ShadowPST, queue_size=20)
    msg = ShadowPST()

    time_start = rospy.get_time()
    while not rospy.is_shutdown():
        time_passed = rospy.get_time() - time_start

        pressure_th = int(np.sin(4*time_passed)*100)+100
        pressure_ff = int(np.sin(4*time_passed+1)*100)+100
        pressure_mf = int(np.sin(6*time_passed+1.8)*100)+100
        pressure_rf = int(np.sin(10*time_passed+2.5)*100)+100
        pressure_lf = int(np.sin(time_passed+0.4)*100)+100

        msg.pressure = [pressure_ff, pressure_mf, pressure_rf, pressure_lf, pressure_th]

        pub.publish(msg)
        rospy.Rate(120).sleep()
