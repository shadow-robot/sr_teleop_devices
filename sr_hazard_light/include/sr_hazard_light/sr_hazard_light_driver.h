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
#include <sr_hazard_light/Status.h>
#include <sr_hazard_light/SetHazardLight.h>

class SrHazardLights
{
    public:
        SrHazardLights();
        ~SrHazardLights();
        void start(int publishing_rate);
        void stop();
        ros::ServiceServer hazard_light_service;

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
        ros::Timer light_timer;
        ros::Timer buzzer_timer;

        ros::Publisher hazard_light_publisher_ = nh_.advertise<sr_hazard_light::Status>("sr_hazard_light/status", 1);
        std::vector<uint8_t> default_buffer = std::vector<uint8_t>(8);
        std::vector<uint8_t> current_buffer = std::vector<uint8_t>(8);

        bool open_device();
        void close_device();
        bool change_hazard_light(sr_hazard_light::SetHazardLight::Request &request,
                                 sr_hazard_light::SetHazardLight::Response &response);
        bool set_light(int pattern, std::string colour, int duration, bool reset);
        bool set_buzzer(int pattern, int tonea, int toneb, int duration, int reset);
        bool send_buffer(std::uint8_t sent_buffer[8]);
        void light_timer_cb(const ros::TimerEvent& event);
        void buzzer_timer_cb(const ros::TimerEvent& event);
        void detect_device_event(libusb_hotplug_event event);
        int on_usb_hotplug(struct libusb_context *ctx,
                           struct libusb_device *device,
                           libusb_hotplug_event event);
        static int on_usb_hotplug_callback(struct libusb_context *ctx,
                                           struct libusb_device *device,
                                           libusb_hotplug_event event,
                                           void* discovery);
        void hotplug_loop();
        void publish_hazard_light_data();
};


#endif  // SR_HAZARD_LIGHT_SR_HAZARD_LIGHT_DRIVER_H
