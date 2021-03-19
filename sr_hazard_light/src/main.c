#include <stdint.h>
#include <stdio.h>
#include "sr_hazard_light/sr_hazard_light_driver.h"

// you will need to create a "patlite.h"
// to be able to call the following functions from outside.
// patlite_lights(), patlite_buzzer(), patlite_init()

int main (void) {

  patlite_init();

  patlite_lights(1, 1, 1, 1, 1);

  return 0;

}