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

#ifndef SR_HAZARD_LIGHT_SR_HAZARD_LIGHT_DRIVER_H
#define SR_HAZARD_LIGHT_SR_HAZARD_LIGHT_DRIVER_H

#include <libusb-1.0/libusb.h>
#include <ros/ros.h>
#include <atomic>
#include <thread>  // NOLINT(build/c++11)
#include <vector>
#include <string>
#include <map>
#include <sr_hazard_light/Status.h>
#include <sr_hazard_light/SetHazardLight.h>
#include <sr_hazard_light/ResetHazardLight.h>

struct hazard_light_data
{
    ros::Timer timer;
    std::vector<uint8_t> buffer = std::vector<uint8_t>(8);
};

class SrHazardLights
{
    public:
        SrHazardLights();
        ~SrHazardLights();
        void start(int publishing_rate);
        void stop();
        ros::ServiceServer hazard_light_service;
        ros::ServiceServer reset_hazard_light_service;

    private:
        ros::NodeHandle nh_ = ros::NodeHandle();

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
        std::map<int16_t, hazard_light_data> red_light_timers;
        std::map<int16_t, hazard_light_data> orange_light_timers;
        std::map<int16_t, hazard_light_data> green_light_timers;
        std::map<int16_t, hazard_light_data> buzzer_timers;
        const int16_t default_setting_key = 0;
        int16_t timer_setting_key = 1;

        ros::Publisher hazard_light_publisher_ = nh_.advertise<sr_hazard_light::Status>("sr_hazard_light/status", 1);

        bool open_device();
        void close_device();
        bool change_hazard_light(sr_hazard_light::SetHazardLight::Request &request,
                                 sr_hazard_light::SetHazardLight::Response &response);
        bool reset_hazard_light(sr_hazard_light::ResetHazardLight::Request &request,
                                 sr_hazard_light::ResetHazardLight::Response &response);
        bool set_light(int pattern, std::string colour, int duration);
        bool update_light(std::map<int16_t, hazard_light_data>& timer_map,
                          std::vector<uint8_t> buffer, int duration);
        bool set_buzzer(int pattern, int tonea, int toneb, int duration);
        bool send_buffer(std::uint8_t sent_buffer[8]);
        void timer_cb(int16_t timer_key_remove, std::map<int16_t, hazard_light_data>* timer_map);
        void publish_hazard_light_data();

        void detect_device_event(libusb_hotplug_event event);
        int on_usb_hotplug(struct libusb_context *ctx,
                           struct libusb_device *device,
                           libusb_hotplug_event event);
        static int on_usb_hotplug_callback(struct libusb_context *ctx,
                                           struct libusb_device *device,
                                           libusb_hotplug_event event,
                                           void* discovery);
        void hotplug_loop();
};


#endif  // SR_HAZARD_LIGHT_SR_HAZARD_LIGHT_DRIVER_H
