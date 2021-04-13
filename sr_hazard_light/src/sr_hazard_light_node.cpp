/*
* Copyright 2021 Shadow Robot Company Ltd.
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
*/

#include "sr_hazard_light/sr_hazard_light_driver.h"
#include <ros/ros.h>
#include <ros/time.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "sr_teleop_hazard_light");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  SrHazardLights sr_hazard_lights;

  int publishing_rate;
  ros::param::param<int>("~publishing_rate", publishing_rate, 1);

  sr_hazard_lights.start(publishing_rate);

  return 0;
}
