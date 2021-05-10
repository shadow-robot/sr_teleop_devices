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



SrHazardLights::SrHazardLights()
  :started_(false), context_(nullptr), connected_(false), detected_(false),
  red_light_(false), orange_light_(false), green_light_(false), buzzer_on_(false),
  default_key(0), timer_key(1)
{
  libusb_init(&context_);

  hazard_light_service = nh_.advertiseService("sr_hazard_light/set_hazard_light",
                         &SrHazardLights::change_hazard_light, this);
  reset_hazard_light_service = nh_.advertiseService("sr_hazard_light/reset_hazard_light",
                         &SrHazardLights::reset_hazard_light, this);

  std::vector<uint8_t> default_red_buffer = {0x00, 0x00, 0xFF, 0xFF, 0x0F, 0xFF, 0x00, 0x00};
  std::vector<uint8_t> default_orange_buffer = {0x00, 0x00, 0xFF, 0xFF, 0xF0, 0xFF, 0x00, 0x00};
  std::vector<uint8_t> default_green_buffer = {0x00, 0x00, 0xFF, 0xFF, 0x0F, 0xFF, 0x00, 0x00};
  std::vector<uint8_t> default_buzzer_buffer = {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00};

  ros::Timer red_default_timer = nh_.createTimer(ros::Duration(10),
                                                 std::bind(&SrHazardLights::timer_cb, this,
                                                 timer_key, &red_light_timers), true, false);
  ros::Timer orange_default_timer = nh_.createTimer(ros::Duration(10),
                                                    std::bind(&SrHazardLights::timer_cb, this,
                                                    timer_key, &orange_light_timers), true, false);
  ros::Timer green_default_timer = nh_.createTimer(ros::Duration(10),
                                                   std::bind(&SrHazardLights::timer_cb, this,
                                                   timer_key, &green_light_timers), true, false);
  ros::Timer buzzer_default_timer = nh_.createTimer(ros::Duration(10),
                                                    std::bind(&SrHazardLights::timer_cb, this,
                                                    timer_key, &buzzer_timers), true, false);

  red_light_timers.insert(std::pair<int16_t, hazard_light_data>(default_key,
                          {red_default_timer, default_red_buffer}));
  orange_light_timers.insert(std::pair<int16_t, hazard_light_data>(default_key,
                             {orange_default_timer, default_orange_buffer}));
  green_light_timers.insert(std::pair<int16_t, hazard_light_data>(default_key,
                            {green_default_timer, default_green_buffer}));
  buzzer_timers.insert(std::pair<int16_t, hazard_light_data>(default_key,
                       {buzzer_default_timer, default_buzzer_buffer}));
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
  libusb_hotplug_deregister_callback(context_, hotplug_callback_handle_);
  started_ = false;
  hotplug_loop_thread_.join();
  uint8_t stop_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  bool retval = send_buffer(stop_buffer);
  libusb_exit(context_);
  ROS_INFO("Closing hazard light device");
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

bool SrHazardLights::open_device()
{
  patlite_handle = libusb_open_device_with_vid_pid(NULL, LIGHT_VID, LIGHT_PID);
  if (patlite_handle == NULL)
  {
    ROS_ERROR("Hazard Light device not found");
    return false;
  }

  libusb_set_auto_detach_kernel_driver(patlite_handle, 1);

  int retval = libusb_claim_interface(patlite_handle, 0);
  if (retval != LIBUSB_SUCCESS)
  {
    ROS_ERROR("libusb claim failed");
    return false;
  }

  uint8_t start_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  bool return_val = send_buffer(start_buffer);

  if (!return_val)
    return return_val;

  connected_ = true;
  ROS_INFO("Hazard light USB device opened and claimed");

  return true;
};

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
    bool set_light_result = set_light(light[light_cmd].pattern,
                                      light[light_cmd].colour,
                                      light[light_cmd].duration);
    if (set_light_result && !complete_commands)
      complete_commands = false;
  }

  for (size_t buzzer_cmd = 0; buzzer_cmd < buzzer.size(); buzzer_cmd++)
  {
    bool set_buzzer_result = set_buzzer(buzzer[buzzer_cmd].pattern,
                                        buzzer[buzzer_cmd].tonea,
                                        buzzer[buzzer_cmd].toneb,
                                        buzzer[buzzer_cmd].duration);
    if (set_buzzer_result && !complete_commands)
      complete_commands = false;
  }

  response.confirmation = complete_commands;
  return response.confirmation;
}


bool SrHazardLights::set_light(int pattern, std::string colour, int duration)
{
  if (pattern > 9 || pattern < 0)
  {
    ROS_ERROR("Number chosen for light pattern is out of range");
    return false;
  }

  if (duration < 0)
  {
    ROS_ERROR("Duration chosen for light pattern is out of range");
    return false;
  }

  std::list<std::string> colours = {"red", "orange", "green"};
  std::vector<uint8_t> buffer = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
  if (std::find(std::begin(colours), std::end(colours), colour) != std::end(colours))
  {
    if (colour == "red")
    {
      buffer[4] = (pattern << 4) | (buffer[4] & 0x0F);
      red_light_ = pattern != 0;
      return update_light(red_light_timers, buffer, duration);

    }
    else if (colour == "orange")
    {
      buffer[4] = (buffer[4] & 0xF0) | pattern;
      orange_light_ = pattern != 0;
      return update_light(orange_light_timers, buffer, duration);
    }
    else if (colour == "green")
    {
      buffer[5] = (pattern << 4) | (buffer[5] & 0x0F);
      green_light_ = pattern != 0;
      return update_light(green_light_timers, buffer, duration);
    }
    else
    {
      ROS_ERROR("Colour sent is not a colour on the hazard light");
      return false;
    }
  }

  return true;
}

bool::SrHazardLights::update_light(std::map<int16_t, hazard_light_data>& timer_map, std::vector<uint8_t> buffer, int duration)
{
  std::uint8_t* sent_buffer = &buffer[0];

  if (duration == 0)
  {
    timer_map[default_key].buffer = buffer;
    if (timer_map.size() == 1)
    {
      bool retval = send_buffer(sent_buffer);
      if (!retval)
        return false;
    }
  }
  else
  {
    bool retval = send_buffer(sent_buffer);
    if (!retval)
      return false;
    ++timer_key;
    ros::Timer light_timer = nh_.createTimer(ros::Duration(duration),
                                             std::bind(&SrHazardLights::timer_cb, this,
                                             timer_key, &timer_map), true, true);
    light_timer.setPeriod(ros::Duration(duration), true);
    light_timer.start();
    timer_map.insert(std::pair<int16_t, hazard_light_data>(timer_key, {light_timer, buffer}));
  }
  return true;
}

bool SrHazardLights::set_buzzer(int pattern, int tonea, int toneb, int duration)
{
  if (pattern < 0 || pattern > 9 || tonea > 15 || toneb > 15 || pattern < 0 || tonea < 0 || toneb < 0 || duration < 0)
  {
    ROS_ERROR("Number or duration chosen for buzzer is out of range");
    return false;
  }

  std::vector<uint8_t> buffer = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
  buffer[2] = (buffer[2] & 0xF0) | pattern;
  buffer[3] = (tonea << 4) + toneb;
  std::uint8_t* sent_buffer = &buffer[0];
  bool retval = send_buffer(sent_buffer);
  if (!retval)
    return false;

  buzzer_on_ = pattern != 0;

  if (duration == 0)
  {
    buzzer_timers[default_key].buffer[2] = buffer[2];
    buzzer_timers[default_key].buffer[3] = buffer[3];
  }
  else
  {
    ++timer_key;
    ros::Timer buzzer_timer = nh_.createTimer(ros::Duration(duration),
                                              std::bind(&SrHazardLights::timer_cb, this,
                                              timer_key, &buzzer_timers), true, true);
    buzzer_timer.setPeriod(ros::Duration(duration), true);
    buzzer_timer.start();
    buzzer_timers.insert(std::pair<int16_t, hazard_light_data>(timer_key, {buzzer_timer, buffer}));
  }
  return true;
}

bool SrHazardLights::send_buffer(std::uint8_t sent_buffer[8])
{
  int retval, rs = 0;
  retval = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT_OUT, sent_buffer, BUFFER_SIZE, &rs, TIMEOUT);
  if (retval)
  {
    ROS_ERROR("Hazard light failed to set with error: %s", libusb_error_name(retval));
    libusb_close(patlite_handle);
    patlite_handle = 0;
    return false;
  }
  return true;
}

void SrHazardLights::timer_cb(int16_t timer_key_remove, std::map<int16_t, hazard_light_data>* timer_map)
{
  std::pair<int16_t, hazard_light_data> reset_data = *(std::prev(timer_map->find(timer_key_remove)));
  if (++timer_map->find(timer_key_remove) == timer_map->end())
  {
    std::uint8_t* reset_buf = &reset_data.second.buffer[0];

    int retval, rs = 0;
    retval = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT_OUT, reset_buf, BUFFER_SIZE, &rs, 1000);
    if (retval)
    {
      ROS_ERROR("Hazard light failed to set with error: %s", libusb_error_name(retval));
      libusb_close(patlite_handle);
      patlite_handle = 0;
    }

    if ((reset_data.second.buffer[4] & 0xF0) == 0x00)
      red_light_ = false;
    else if ((reset_data.second.buffer[4] & 0x0F) == 0x00)
      orange_light_ = false;
    else if ((reset_data.second.buffer[5] & 0xF0) == 0x00)
      green_light_ = false;
    else if ((reset_data.second.buffer[2] & 0x0F) == 0x00)
      buzzer_on_ = false;

  }
  timer_map->erase(timer_key_remove);
}

bool SrHazardLights::reset_hazard_light(sr_hazard_light::ResetHazardLight::Request &request,
                                        sr_hazard_light::ResetHazardLight::Response &response)
{
  ROS_INFO("Resetting hazard light.");
  uint8_t reset_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  red_light_timers.clear();
  orange_light_timers.clear();
  orange_light_timers.clear();
  buzzer_timers.clear();
  red_light_ = false;
  orange_light_ = false;
  green_light_ = false;
  buzzer_on_ = false;
  response.confirmation = send_buffer(reset_buffer);

  return response.confirmation;
}

void SrHazardLights::close_device()
{
  connected_ = false;
  red_light_ = false;
  orange_light_ = false;
  green_light_ = false;
  buzzer_on_ = false;
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

void SrHazardLights::publish_hazard_light_data()
{
  sr_hazard_light::Status sr_hazard_light_status;
  sr_hazard_light_status.header.stamp = ros::Time::now();
  sr_hazard_light_status.connected = connected_;
  sr_hazard_light_status.red_light_on = red_light_;
  sr_hazard_light_status.orange_light_on = orange_light_;
  sr_hazard_light_status.green_light_on = green_light_;
  sr_hazard_light_status.buzzer_on = buzzer_on_;
  hazard_light_publisher_.publish(sr_hazard_light_status);
}
