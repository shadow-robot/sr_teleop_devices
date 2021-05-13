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
#include <list>
#include <vector>
#include <string>
#include <iterator>
#include <map>
#include <utility>
#include <sr_hazard_light/SetLight.h>
#include <sr_hazard_light/SetBuzzer.h>

#define LIGHT_VID 0x191A
#define LIGHT_PID 0x8003
#define PATLITE_ENDPOINT_OUT 1
#define PATLITE_ENDPOINT_IN 0x81
#define TIMEOUT 1000
#define BUFFER_SIZE 8

static libusb_device_handle *patlite_handle = 0;

HazardEvent::HazardEvent(ros::NodeHandle node_handle_, std::vector<uint8_t> buffer)
            :nh_(node_handle_), buffer_default(buffer)
{
  default_timer = nh_.createTimer(ros::Duration(10), std::bind(&HazardEvent::timer_cb, this,
                                  timed_setting_key, &timer_map), true, false);
  timer_map.insert(std::pair<int16_t, hazard_light_data>(default_setting_key,
                    {default_timer, buffer}));
}

bool HazardEvent::send_buffer(std::uint8_t sent_buffer[8])
{
  int rs = 0;
  int retval = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT_OUT, sent_buffer, BUFFER_SIZE, &rs, TIMEOUT);
  if (retval)
  {
    ROS_ERROR("Hazard light failed to set with error: %s", libusb_error_name(retval));
    libusb_close(patlite_handle);
    patlite_handle = 0;
    return false;
  }
  return true;
}

void HazardEvent::timer_cb(int16_t timer_key_remove, std::map<int16_t, hazard_light_data>* timer_map)
{
  std::pair<int16_t, hazard_light_data> reset_data = *(std::prev(timer_map->find(timer_key_remove)));
  if (++timer_map->find(timer_key_remove) == timer_map->end())
  {
    std::uint8_t* reset_buf = &reset_data.second.buffer[0];

    int rs = 0;
    int retval = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT_OUT, reset_buf, BUFFER_SIZE, &rs, 1000);
    if (retval)
    {
      ROS_ERROR("Hazard light failed to set with error: %s", libusb_error_name(retval));
      libusb_close(patlite_handle);
      patlite_handle = 0;
    }

    change_topic_bool(reset_data.second.buffer);
  }
  timer_map->erase(timer_key_remove);
}

void HazardEvent::clear()
{
  is_on = false;
  timed_setting_key = 1;
  timer_map.clear();
  timer_map.insert(std::pair<int16_t, hazard_light_data>(default_setting_key,
                         {default_timer, buffer_default}));
}

bool HazardLight::update_light(std::vector<uint8_t> buffer, int duration)
{
  std::uint8_t* sent_buffer = &buffer[0];
  if (duration == 0)
  {
    timer_map[default_setting_key].buffer = buffer;
    if (timer_map.size() == 1)
    {
      if (!send_buffer(sent_buffer)) return false;
    }
  }
  else
  {
    if (!send_buffer(sent_buffer)) return false;
    ++timed_setting_key;
    ros::Timer light_timer = nh_.createTimer(ros::Duration(duration),
                                             std::bind(&HazardEvent::timer_cb, this,
                                             timed_setting_key, &timer_map), true, true);
    light_timer.setPeriod(ros::Duration(duration), true);
    light_timer.start();
    timer_map.insert(std::pair<int16_t, hazard_light_data>(timed_setting_key, {light_timer, buffer}));
  }
  return true;
}

bool HazardBuzzer::set_buzzer(int pattern, int tonea, int toneb, int duration)
{
  std::vector<uint8_t> buffer = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
  buffer[2] = (buffer[2] & 0xF0) | pattern;
  buffer[3] = (tonea << 4) + toneb;
  std::uint8_t* sent_buffer = &buffer[0];
  if (!send_buffer(sent_buffer)) return false;

  is_on = pattern != 0;

  if (duration == 0)
  {
    timer_map[default_setting_key].buffer[2] = buffer[2];
    timer_map[default_setting_key].buffer[3] = buffer[3];
  }
  else
  {
    ++timed_setting_key;
    ros::Timer buzzer_timer = nh_.createTimer(ros::Duration(duration),
                                              std::bind(&HazardEvent::timer_cb, this,
                                              timed_setting_key, &timer_map), true, true);
    buzzer_timer.setPeriod(ros::Duration(duration), true);
    buzzer_timer.start();
    timer_map.insert(std::pair<int16_t, hazard_light_data>(timed_setting_key, {buzzer_timer, buffer}));
  }
  return true;
}

void HazardBuzzer::change_topic_bool(std::vector<uint8_t> buffer)
{
  if ((buffer[2] & 0x0F) == 0x00)
    is_on = false;
}

bool HazardLightRed::set_light(int pattern, int duration, std::vector<uint8_t> buffer)
{
  buffer[4] = (pattern << 4) | (buffer[4] & 0x0F);
  is_on = pattern != 0;
  return update_light(buffer, duration);
}

void HazardLightRed::change_topic_bool(std::vector<uint8_t> buffer)
{
  if ((buffer[4] & 0xF0) == 0x00)
    is_on = false;
}

bool HazardLightOrange::set_light(int pattern, int duration, std::vector<uint8_t> buffer)
{
  buffer[4] = (buffer[4] & 0xF0) | pattern;
  is_on = pattern != 0;
  return update_light(buffer, duration);
}

void HazardLightOrange::change_topic_bool(std::vector<uint8_t> buffer)
{
  if ((buffer[4] & 0x0F) == 0x00)
    is_on = false;
}

bool HazardLightGreen::set_light(int pattern, int duration, std::vector<uint8_t> buffer)
{
  buffer[5] = (pattern << 4) | (buffer[5] & 0x0F);
  is_on = pattern != 0;
  return update_light(buffer, duration);
}

void HazardLightGreen::change_topic_bool(std::vector<uint8_t> buffer)
{
  if ((buffer[5] & 0xF0) == 0x00)
    is_on = false;
}

SrHazardLights::SrHazardLights()
  :started_(false), context_(nullptr), connected_(false), detected_(false)
{
  libusb_init(&context_);

  hazard_light_service = nh_.advertiseService("sr_hazard_light/set_hazard_light",
                         &SrHazardLights::change_hazard_light, this);
  reset_hazard_light_service = nh_.advertiseService("sr_hazard_light/reset_hazard_light",
                         &SrHazardLights::reset_hazard_light, this);
}

SrHazardLights::~SrHazardLights()
{
  stop();
}

void SrHazardLights::start(int publishing_rate)
{
  publishing_rate_ = publishing_rate;

  if (!started_)
  {
    int ret = libusb_hotplug_register_callback(context_,
            (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
            LIBUSB_HOTPLUG_ENUMERATE, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
            &SrHazardLights::on_usb_hotplug_callback, reinterpret_cast<void*>(this), &hotplug_callback_handle_);

    if (LIBUSB_SUCCESS == ret)
      started_ = true;

    // start separate thread to detect hotplug events
    hotplug_loop_thread_ = std::thread(&SrHazardLights::hotplug_loop, this);

    while (ros::ok() && started_)
    {
      if (detected_)
      {
        if (!connected_)
          open_device();
      }
      else
        close_device();

      publish_hazard_light_data();
      ros::Rate(publishing_rate_).sleep();
      ros::spinOnce();
    }
  }
}

void SrHazardLights::stop()
{
  ROS_INFO("Closing hazard light device");
  libusb_hotplug_deregister_callback(context_, hotplug_callback_handle_);
  started_ = false;
  hotplug_loop_thread_.join();
  uint8_t stop_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  bool retval = HazardEvent::send_buffer(stop_buffer);
  libusb_exit(context_);
}

bool SrHazardLights::open_device()
{
  patlite_handle = libusb_open_device_with_vid_pid(NULL, LIGHT_VID, LIGHT_PID);
  if (patlite_handle == NULL)
  {
    ROS_ERROR("Hazard Light device not found");
    return false;
  }

  libusb_set_auto_detach_kernel_driver(patlite_handle, 1);

  if (libusb_claim_interface(patlite_handle, 0) != LIBUSB_SUCCESS)
  {
    ROS_ERROR("libusb claim failed");
    return false;
  }

  uint8_t start_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (!HazardEvent::send_buffer(start_buffer)) return false;

  connected_ = true;
  red_lights.is_on = false;
  orange_lights.is_on = false;
  green_lights.is_on = false;
  buzzers.is_on = false;

  ROS_INFO("Hazard light USB device opened and claimed");

  return true;
};


void SrHazardLights::close_device()
{
  connected_ = false;
  red_lights.is_on = false;
  orange_lights.is_on = false;
  green_lights.is_on = false;
  buzzers.is_on = false;
}

bool SrHazardLights::change_hazard_light(sr_hazard_light::SetHazardLight::Request &request,
                                      sr_hazard_light::SetHazardLight::Response &response)
{
  std::vector<sr_hazard_light::SetLight> light = request.light;
  std::vector<sr_hazard_light::SetBuzzer> buzzer = request.buzzer;

  if (!patlite_handle)
  {
    if (!SrHazardLights::open_device())
    {
      ROS_ERROR("Hazard light set failed, return false");
      return false;
    }
  }

  bool complete_commands = true;
  for (size_t light_cmd = 0; light_cmd < light.size(); light_cmd++)
  {
    if (light[light_cmd].pattern < 0 || light[light_cmd].pattern > 9 || light[light_cmd].pattern < 0)
    {
      ROS_ERROR("Number chosen for buzzer pattern is out of range");
      return false;
    }

    if (light[light_cmd].duration < 0)
    {
      ROS_ERROR("Number chosen for buzzer duration is out of range");
      return false;
    }

    bool set_light_result;
    std::vector<uint8_t> buffer = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
    std::list<std::string> colours = {"red", "orange", "green"};
    if (std::find(std::begin(colours), std::end(colours), light[light_cmd].colour) != std::end(colours))
    {
      if (light[light_cmd].colour == "red")
        set_light_result = red_lights.set_light(light[light_cmd].pattern,
                                                light[light_cmd].duration,
                                                buffer);
      else if (light[light_cmd].colour == "orange")
        set_light_result = orange_lights.set_light(light[light_cmd].pattern,
                                                   light[light_cmd].duration,
                                                   buffer);
      else if (light[light_cmd].colour == "green")
        set_light_result = green_lights.set_light(light[light_cmd].pattern,
                                                  light[light_cmd].duration,
                                                  buffer);
    }
    else
    {
      ROS_ERROR("Colour sent is not a colour on the hazard light");
      set_light_result = false;
    }
    if (set_light_result && !complete_commands)
      complete_commands = false;
  }

  for (size_t buzzer_cmd = 0; buzzer_cmd < buzzer.size(); buzzer_cmd++)
  {
    if (buzzer[buzzer_cmd].pattern < 0 || buzzer[buzzer_cmd].pattern > 9
        || buzzer[buzzer_cmd].pattern < 0)
    {
      ROS_ERROR("Number chosen for buzzer pattern is out of range");
      return false;
    }

    if (buzzer[buzzer_cmd].tonea > 15 || buzzer[buzzer_cmd].toneb > 15
        || buzzer[buzzer_cmd].tonea < 0 || buzzer[buzzer_cmd].toneb < 0)
    {
      ROS_ERROR("Number chosen for buzzer tone is out of range");
      return false;
    }

    if (buzzer[buzzer_cmd].duration < 0)
    {
      ROS_ERROR("Number chosen for buzzer duration is out of range");
      return false;
    }
    bool set_buzzer_result = buzzers.set_buzzer(buzzer[buzzer_cmd].pattern,
                                        buzzer[buzzer_cmd].tonea,
                                        buzzer[buzzer_cmd].toneb,
                                        buzzer[buzzer_cmd].duration);
    if (set_buzzer_result && !complete_commands)
      complete_commands = false;
  }

  response.confirmation = complete_commands;
  return response.confirmation;
}

bool SrHazardLights::reset_hazard_light(sr_hazard_light::ResetHazardLight::Request &request,
                                        sr_hazard_light::ResetHazardLight::Response &response)
{
  ROS_INFO("Resetting hazard light.");
  uint8_t reset_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  red_lights.clear();
  orange_lights.clear();
  green_lights.clear();
  buzzers.clear();
  response.confirmation = HazardEvent::send_buffer(reset_buffer);
  return response.confirmation;
}

void SrHazardLights::publish_hazard_light_data()
{
  sr_hazard_light::Status sr_hazard_light_status;
  sr_hazard_light_status.header.stamp = ros::Time::now();
  sr_hazard_light_status.connected = connected_;
  sr_hazard_light_status.red_light_on = red_lights.is_on;
  sr_hazard_light_status.orange_light_on = orange_lights.is_on;
  sr_hazard_light_status.green_light_on = green_lights.is_on;
  sr_hazard_light_status.buzzer_on = buzzers.is_on;
  hazard_light_publisher_.publish(sr_hazard_light_status);
}

void SrHazardLights::detect_device_event(libusb_hotplug_event event)
{
  switch (event)
  {
    case LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED:
      ROS_INFO("Hazard light device detected");
      detected_ = true;
      break;
    case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT:
      ROS_INFO("Hazard light disconnected");
      detected_ = false;
      break;
    default:
      ROS_WARN("Unknown event detected");
      break;
  }
}

int SrHazardLights::on_usb_hotplug(struct libusb_context *ctx,
                                   struct libusb_device *device,
                                   libusb_hotplug_event event)
{
  struct libusb_device_descriptor descriptor;

  int ret = libusb_get_device_descriptor(device, &descriptor);
  if (LIGHT_VID == descriptor.idVendor && LIGHT_PID == descriptor.idProduct)
  {
    if (LIBUSB_SUCCESS == ret)
    {
      detect_device_event(event);
    }
    else
    {
      ROS_ERROR("Got a device but failed to look up its descriptor");
    }
  }
  return 0;
}

int SrHazardLights::on_usb_hotplug_callback(struct libusb_context *ctx,
                                           struct libusb_device *device,
                                           libusb_hotplug_event event,
                                           void* light_discovery)
                                           {
  return (reinterpret_cast<SrHazardLights*>(light_discovery))->on_usb_hotplug(ctx, device, event);
}

void SrHazardLights::hotplug_loop()
{
  while (started_)
  {
    libusb_handle_events_completed(context_, nullptr);
    ros::Rate(publishing_rate_).sleep();
  }
}
