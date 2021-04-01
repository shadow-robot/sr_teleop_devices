#ifndef SR_HAZARD_LIGHTS_SR_HAZARD_LIGHTS_H
#define SR_HAZARD_LIGHTS_SR_HAZARD_LIGHTS_H

#include <libusb-1.0/libusb.h>
#include <iostream>
#include <cstdlib>
#include <cstdint>
#include <ros/ros.h>
#include <atomic>
#include <thread>  // NOLINT(build/c++11)
#include <sr_hazard_light/Status.h>
#include <hidapi/hidapi.h>
#include <sr_hazard_light/SetLight.h>
#include <sr_hazard_light/SetBuzzer.h>

class SrHazardLights
{
    public:
        SrHazardLights();
        ~SrHazardLights();
        void start(int publishing_rate);
        void stop();
        // hid_device *device_handle_;
        ros::NodeHandle nh_ = ros::NodeHandle();

        ros::ServiceServer hazard_light_service;
        ros::ServiceServer hazard_buzzer_service;

        bool set_light(sr_hazard_light::SetLight::Request &request, 
                      sr_hazard_light::SetLight::Response &response);
        bool set_buzzer(sr_hazard_light::SetBuzzer::Request &request, 
                       sr_hazard_light::SetBuzzer::Response &response);
        bool set(int duration, std::uint8_t buf[8]);

        libusb_context *context_;
        bool started_;
        int publishing_rate_;
        bool connected_;
        std::atomic<bool> detected_;
        libusb_hotplug_callback_handle hotplug_callback_handle_;
        std::thread hotplug_loop_thread_;
        bool red_light_;
        bool orange_light_;
        bool green_light_;
        bool buzzer_on_;

        ros::Publisher hazard_light_publisher_ = nh_.advertise<sr_hazard_light::Status>("sr_hazard_light/status", 1);
        std::vector<uint8_t> buffer_ = std::vector<uint8_t>(8);

        bool open_device();
        void close_device();
        void detect_device_event(libusb_hotplug_event event);
        // void read_data_from_device();
        int on_usb_hotplug(struct libusb_context *ctx,
                            struct libusb_device *device,
                            libusb_hotplug_event event);
        static int on_usb_hotplug_callback(struct libusb_context *ctx,
                                    struct libusb_device *device,
                                    libusb_hotplug_event event,
                                    void* discovery);
        void hotplug_loop();
        void publish_hazard_light_data();
        void initServer();
};

#endif //  SR_HAZARD_LIGHTS_SR_HAZARD_LIGHTS_H