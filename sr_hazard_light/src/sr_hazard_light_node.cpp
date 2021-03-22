// #include <cstdlib>
#include "sr_hazard_light/sr_hazard_light_driver.h"
#include <ros/ros.h>
#include <ros/time.h>


// you will need to create a "patlite.h"
// to be able to call the following functions from outside.
// patlite_lights(), patlite_buzzer(), patlite_init()

int main (void) {

  SrHazardLights sr_hazard_lights;

  sr_hazard_lights.patlite_init();

  sr_hazard_lights.patlite_lights(1, 1, 1, 0, 0);

  ros::Duration(1, 0).sleep();
  
  sr_hazard_lights.patlite_lights(1, 0, 0, 0, 0);

  return 0;

}