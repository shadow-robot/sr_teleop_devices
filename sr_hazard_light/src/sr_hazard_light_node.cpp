// #include <cstdlib>
#include "sr_hazard_light/sr_hazard_light_driver.h"
#include <ros/ros.h>
#include <ros/time.h>

// you will need to create a "patlite.h"
// to be able to call the following functions from outside.
// patlite_lights(), patlite_buzzer(), patlite_init()

int main (int argc, char** argv) {

  ros::init(argc, argv, "sr_teleop_hazard_light");

  SrHazardLights sr_hazard_lights;

  int publishing_rate;
  ros::param::param<int>("~publishing_rate", publishing_rate, 20);

  sr_hazard_lights.start(publishing_rate);

  // sr_hazard_lights.open_device();

  // sr_hazard_lights.patlite_lights(1, 1, 1, 1, 1);

  // // sr_hazard_lights.patlite_buzzer(4, 2, 5);

  // ros::Duration(2, 0).sleep();
  
  // sr_hazard_lights.patlite_lights(1, 0, 0, 0, 0);

  // ros::Duration(2, 0).sleep();
  
  // sr_hazard_lights.patlite_lights(0, 0, 1, 0, 0);

  // ros::Duration(2, 0).sleep();
  
  // sr_hazard_lights.patlite_lights(0, 1, 1, 0, 0);

  return 0;

}