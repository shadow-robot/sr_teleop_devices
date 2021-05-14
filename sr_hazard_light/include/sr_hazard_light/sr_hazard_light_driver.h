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

class HazardEvent
{
    public:
        HazardEvent(ros::NodeHandle node_handle_, std::vector<uint8_t> buffer);
        ~HazardEvent() {}
        bool is_on_;

        static bool send_buffer(std::uint8_t sent_buffer[8]);
        void timer_cb(int16_t timer_key_remove, std::map<int16_t, hazard_light_data>* timer_map_);
        void clear();

    protected:
        std::map<int16_t, hazard_light_data> timer_map_;
        std::vector<uint8_t> buffer_default_;
        ros::Timer default_timer_;
        const int16_t default_setting_key_ = 0;
        int16_t timed_setting_key_ = 1;
        ros::NodeHandle nh_;

        virtual void change_topic_bool(std::vector<uint8_t> buffer) = 0;
};

class HazardLight : public HazardEvent
{
    public:
        HazardLight(ros::NodeHandle node_handle_, std::vector<uint8_t> buffer) :
                       HazardEvent(node_handle_, buffer) {}
        ~HazardLight() {}
        virtual bool set_light(int pattern,  int duration, std::vector<uint8_t> buffer) = 0;

    protected:
        bool update_light(std::vector<uint8_t> buffer, int duration);
};

class HazardBuzzer : public HazardEvent
{
    public:
        explicit HazardBuzzer(ros::NodeHandle node_handle_) :
                       HazardEvent(node_handle_,
                                   {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00}) {}
        ~HazardBuzzer() {}
        bool set_buzzer(int pattern, int tonea, int toneb, int duration);

    protected:
        void change_topic_bool(std::vector<uint8_t> buffer);
};

class HazardLightRed : public HazardLight
{
    public:
        explicit HazardLightRed(ros::NodeHandle node_handle_) :
                       HazardLight(node_handle_, {0x00, 0x00, 0xFF, 0xFF, 0x0F, 0xFF, 0x00, 0x00}) {}
        ~HazardLightRed() {}
        bool set_light(int pattern, int duration, std::vector<uint8_t> buffer);

    protected:
        void change_topic_bool(std::vector<uint8_t> buffer);
};

class HazardLightOrange : public HazardLight
{
    public:
        explicit HazardLightOrange(ros::NodeHandle node_handle_) :
                       HazardLight(node_handle_, {0x00, 0x00, 0xFF, 0xFF, 0xF0, 0xFF, 0x00, 0x00}) {}
        ~HazardLightOrange() {}
        bool set_light(int pattern, int duration, std::vector<uint8_t> buffer);

    protected:
        void change_topic_bool(std::vector<uint8_t> buffer);
};

class HazardLightGreen : public HazardLight
{
    public:
        explicit HazardLightGreen(ros::NodeHandle node_handle_) :
                       HazardLight(node_handle_, {0x00, 0x00, 0xFF, 0xFF, 0x0F, 0xFF, 0x00, 0x00}) {}
        ~HazardLightGreen() {}
        bool set_light(int pattern, int duration, std::vector<uint8_t> buffer);

    protected:
        void change_topic_bool(std::vector<uint8_t> buffer);
};

class SrHazardLights
{
    public:
        SrHazardLights();
        ~SrHazardLights();
        void start(int publishing_rate);
        void stop();

    private:
        ros::NodeHandle nh_ = ros::NodeHandle();

        libusb_context *context_;
        bool started_;
        int publishing_rate_;
        bool connected_;
        std::atomic<bool> detected_;
        libusb_hotplug_callback_handle hotplug_callback_handle_;
        std::thread hotplug_loop_thread_;

        HazardLightRed red_lights = HazardLightRed(nh_);
        HazardLightOrange orange_lights = HazardLightOrange(nh_);
        HazardLightGreen green_lights = HazardLightGreen(nh_);
        HazardBuzzer buzzers = HazardBuzzer(nh_);

        ros::Publisher hazard_light_publisher_ = nh_.advertise<sr_hazard_light::Status>("sr_hazard_light/status", 1);
        ros::ServiceServer hazard_light_service_  = nh_.advertiseService("sr_hazard_light/set_hazard_light",
                                                                         &SrHazardLights::change_hazard_light, this);
        ros::ServiceServer reset_hazard_light_service_  = nh_.advertiseService("sr_hazard_light/reset_hazard_light",
                                                                               &SrHazardLights::reset_hazard_light, this);

        bool open_device();
        void close_device();
        bool change_hazard_light(sr_hazard_light::SetHazardLight::Request &request,
                                 sr_hazard_light::SetHazardLight::Response &response);
        bool reset_hazard_light(sr_hazard_light::ResetHazardLight::Request &request,
                                 sr_hazard_light::ResetHazardLight::Response &response);
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
