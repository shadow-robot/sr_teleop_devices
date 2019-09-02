/**
* @file   shadowhand_to_cybergrasp_remapper_node.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Thu May 13 10:39:44 2010
*
*
/* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @brief Launch a ros node to remap data coming from the Dextrous Hand to the Cybergrasp.
*
*
*/

#include <ros/ros.h>

#include "sr_remappers/shadowhand_to_cybergrasp_remapper.h"

using namespace ros;
using namespace shadowhand_to_cybergrasp_remapper;

  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////


int main(int argc, char** argv)
{
  ros::init(argc, argv, "shadowhand_listener");

  ShadowhandToCybergraspRemapper remapper;
  ros::spin();

  return 0;
}
