#ifndef SR_HAZARD_LIGHTS_SR_HAZARD_LIGHTS_H
#define SR_HAZARD_LIGHTS_SR_HAZARD_LIGHTS_H

#include <libusb-1.0/libusb.h>
#include <iostream>
#include <cstdlib>
#include <cstdint>
#include <ros/ros.h>
#include <atomic>
#include <thread>  // NOLINT(build/c++11)
// #include <sr_hazard_light/Status.h>


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
        SrHazardLights();
        ~SrHazardLights();
        void start(int publishing_rate);
        void stop();
        ros::NodeHandle nh_ = ros::NodeHandle();
        // int patlite_lights(int red, int yellow, int green, int blue, int clear);
        // int patlite_buzzer(int type, int tonea, int toneb);
        // int patlite_set(std::uint8_t *buf);

        libusb_context *context_;
        bool started_;
        int publishing_rate_;
        bool connected_;
        std::atomic<bool> detected_;
        libusb_hotplug_callback_handle hotplug_callback_handle_;
        std::thread hotplug_loop_thread_;

        // ros::Publisher hazard_light_publisher_ = nh_.advertise<sr_hazard_light::Status>("sr_hazard_light/status", 1);
        unsigned char buffer_[8];

        int open_device();
        void close_device();
        int on_usb_hotplug(struct libusb_context *ctx,
                            struct libusb_device *device,
                            libusb_hotplug_event event);
        static int on_usb_hotplug_callback(struct libusb_context *ctx,
                                    struct libusb_device *device,
                                    libusb_hotplug_event event,
                                    void* discovery);
        void hotplug_loop();
};

#endif //  SR_HAZARD_LIGHTS_SR_HAZARD_LIGHTS_H