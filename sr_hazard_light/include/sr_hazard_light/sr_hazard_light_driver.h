#ifndef SR_HAZARD_LIGHTS_SR_HAZARD_LIGHTS_H
#define SR_HAZARD_LIGHTS_SR_HAZARD_LIGHTS_H

#include <libusb-1.0/libusb.h>
#include <iostream>
#include <cstdlib>
#include <cstdint>
#include <ros/ros.h>


#define PATLITE_VID 0x191A
#define PATLITE_PID 0x8003
#define PATLITE_ENDPOINT 1

/*
 * Patlite is controlled with USB interrrupt OUT messages.
 * 8 bytes of data which contains status for all lights&buzzer.
 *
 * Nibbles:
 *  0  always zero
 *  1  always zero
 *  2  always zero
 *  3  always zero
 *  4  always zero
 *  5  buzzer type (0 off, 1-5 on/patterns, 8 no change)
 *  6  buzzer tone a (0 none, 1-13 on: 1760Hz - 3520Hz, 15 no change)
 *  7  buzzer tone b (0 none, 1-13 on: 1760Hz - 3520Hz, 15 no change)
 *  8     red light (0 off, 1 static, 2-5 patterns, 8 no change)
 *  9  yellow light (0 off, 1 static, 2-5 patterns, 8 no change)
 * 10   green light (0 off, 1 static, 2-5 patterns, 8 no change)
 * 11    blue light (0 off, 1 static, 2-5 patterns, 8 no change)
 * 12   clear light (0 off, 1 static, 2-5 patterns, 8 no change)
 * 13  always zero
 * 14  always zero
 * 15  always zero
 *
 * tone Hz increases by 5.9463%
 */

class SrHazardLights
{
    public:
        SrHazardLights() {};
        ~SrHazardLights() {};
        ros::NodeHandle nh_ = ros::NodeHandle();
        int patlite_lights(int red, int yellow, int green, int blue, int clear);
        int patlite_buzzer(int type, int tonea, int toneb);
        int patlite_set(std::uint8_t *buf);
        int patlite_init();

};

#endif //  SR_HAZARD_LIGHTS_SR_HAZARD_LIGHTS_H