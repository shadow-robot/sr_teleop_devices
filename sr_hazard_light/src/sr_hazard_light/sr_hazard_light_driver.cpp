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
  red_light_(false), orange_light_(false), green_light_(false), buzzer_on_(false)
{
  libusb_init(&context_);

  hazard_light_service = nh_.advertiseService("sr_hazard_light/set_hazard_light",
                         &SrHazardLights::change_hazard_light, this);
  reset_hazard_light_service = nh_.advertiseService("sr_hazard_light/reset_hazard_light",
                         &SrHazardLights::reset_hazard_light, this);

  // might have to send a blank buffer at start?

  default_buffer = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  current_buffer = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  timer_key = 1;

  std::vector<uint8_t> default_buzzer_buffer = {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00};
  std::vector<uint8_t> default_red_buffer = {0x00, 0x00, 0xFF, 0xFF, 0x0F, 0xFF, 0x00, 0x00};
  std::vector<uint8_t> default_orange_buffer = {0x00, 0x00, 0xFF, 0xFF, 0xF0, 0xFF, 0x00, 0x00};
  std::vector<uint8_t> default_green_buffer = {0x00, 0x00, 0xFF, 0xFF, 0x0F, 0xFF, 0x00, 0x00};

  ros::Timer buzzer_default_timer = nh_.createTimer(ros::Duration(10), std::bind(&SrHazardLights::buzzer_timer_cb, this, timer_key), true, false);
  ros::Timer red_default_timer = nh_.createTimer(ros::Duration(10), std::bind(&SrHazardLights::light_timer_cb, this, timer_key, red_light_timers), true, false);
  // ros::Timer orange_default_timer = nh_.createTimer(ros::Duration(10), std::bind(&SrHazardLights::orange_timer_cb, this, timer_key), true, false);
  // ros::Timer green_default_timer = nh_.createTimer(ros::Duration(10), std::bind(&SrHazardLights::green_timer_cb, this, timer_key), true, false);

  red_light_timers.insert(std::pair<long, hazard_light_data>(default_key, {red_default_timer, default_red_buffer}));
  // orange_light_timers.insert(std::pair<long, hazard_light_data>(default_key, {orange_default_timer, default_orange_buffer}));
  // green_light_timers.insert(std::pair<long, hazard_light_data>(default_key, {green_default_timer, default_green_buffer}));
  buzzer_timers.insert(std::pair<long, hazard_light_data>(default_key, {buzzer_default_timer, default_buzzer_buffer}));
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
    int ret;
    ret = libusb_hotplug_register_callback(context_,
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
  default_buffer = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  current_buffer = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::uint8_t* buffer = &default_buffer[0];
  int retval, rs = 0;
  retval = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT_OUT, buffer, BUFFER_SIZE, &rs, TIMEOUT);
  if (retval)
  {
    ROS_ERROR("Hazard light failed to set with error: %s\n", libusb_error_name(retval));
    libusb_close(patlite_handle);
    patlite_handle = 0;
  }
  libusb_exit(context_);
  ROS_INFO("Closing hazard light device");
  exit(0);
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
    ROS_ERROR("Hazard Light device not found\n");
    return false;
  }

  libusb_set_auto_detach_kernel_driver(patlite_handle, 1);

  int retval;
  retval = libusb_claim_interface(patlite_handle, 0);
  if (retval != LIBUSB_SUCCESS)
  {
    ROS_ERROR("libusb claim failed\n");
    return false;
  }

  int rs = 0;
  std::uint8_t* buffer = &default_buffer[0];
  retval = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT_OUT, buffer, BUFFER_SIZE, &rs, TIMEOUT);

  if (retval != 0)
  {
    ROS_ERROR("Hazard light reset failed, return %s , transferred %i\n", libusb_error_name(retval), rs);
    libusb_close(patlite_handle);
    patlite_handle = 0;
    return false;
  }

  connected_ = true;
  ROS_INFO("Hazard light USB device opened and claimed\n");

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
      ROS_ERROR("Hazard light set failed, return false\n");
      return false;
    }
  }

  bool complete_commands = true;
  for (size_t light_cmd = 0; light_cmd < light.size(); light_cmd++) 
  {
    bool set_light_result = set_light(light[light_cmd].pattern,
                                      light[light_cmd].colour,
                                      light[light_cmd].duration,
                                      light[light_cmd].reset);
    if (set_light_result && !complete_commands)
      complete_commands = false;
  }

  for(size_t buzzer_cmd = 0; buzzer_cmd < buzzer.size(); buzzer_cmd++)
  {
    bool set_buzzer_result = set_buzzer(buzzer[buzzer_cmd].pattern,
                                        buzzer[buzzer_cmd].tonea,
                                        buzzer[buzzer_cmd].toneb,
                                        buzzer[buzzer_cmd].duration,
                                        buzzer[buzzer_cmd].reset);
    if (set_buzzer_result && !complete_commands)
      complete_commands = false;
  }

  response.confirmation = complete_commands;
  return response.confirmation;
}


bool SrHazardLights::set_light(int pattern, std::string colour, int duration, bool reset)
{
  if (pattern > 9 || pattern < 0 || duration < 0)
  {
    ROS_ERROR("Number or duration chosen for light pattern is out of range");
    return false;
  }

  if (reset == true)
  {
      default_buffer[4] = 0x00;
      default_buffer[5] = 0x00;
      red_light_ = false;
      orange_light_ = false;
      green_light_ = false;
  }

  std::list<std::string> colours = {"red", "orange", "green"};
  std::vector<uint8_t> buffer = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
  if (std::find(std::begin(colours), std::end(colours), colour) != std::end(colours))
  {
    if (colour == "red")
    {
      return update_red_light(pattern, duration, reset);
      buffer[4] = (pattern << 4) | (buffer[4] & 0x0F);
      red_light_ = true;
    }
    else if (colour == "orange")
    {
      buffer[4] = (buffer[4] & 0xF0) | pattern;
      orange_light_ = true;
    }
    else if (colour == "green")
    {
      buffer[5] = (pattern << 4) | (buffer[5] & 0x0F);
      green_light_ = true;
    }
    else
    {
      ROS_ERROR("Colour sent is not a colour on the hazard light");
      return false;
    }
  }

  // if (duration == 0)
  // {
  //   default_buffer[4] = current_buffer[4];
  //   default_buffer[5] = current_buffer[5];
  //   std::uint8_t* sent_buffer = &default_buffer[0];
  //   ROS_ERROR("default light");
  //   ROS_INFO_STREAM("light default buffer: \n");
  //   for (int i = 0; i < 8; i++)
  //   {
  //     ROS_INFO_STREAM(static_cast<int16_t>(sent_buffer[i]) << ", ");
  //   }
  //   ROS_INFO_STREAM("\n");
  //   return send_buffer(sent_buffer);
  // }
  // else
  // {
  //   std::uint8_t* sent_buffer = &current_buffer[0];
  //   int retval;
  //   ROS_INFO_STREAM("light CURRENT buffer: \n");
  //   for (int i = 0; i < 8; i++)
  //   {
  //     ROS_INFO_STREAM(static_cast<int16_t>(sent_buffer[i]) << ", ");
  //   }
  //   ROS_INFO_STREAM("\n");
  //   retval = send_buffer(sent_buffer);
  //   if (retval)
  //   {
  //     ROS_ERROR("starting light timer");
  //     ++timer_key;
  //     ROS_INFO_STREAM("map size:" << buzzer_timers.size());
  //     ROS_INFO_STREAM("timer key:" << timer_key);
  //     ros::Timer light_timer = nh_.createTimer(ros::Duration(duration), std::bind(&SrHazardLights::light_timer_cb, this, timer_key, light_timers), true, true);
  //     light_timer.setPeriod(ros::Duration(duration), true);
  //     light_timer.start();
  //     light_timers.insert(std::pair<long,ros::Timer>(timer_key, light_timer));
  //     ROS_INFO_STREAM("map size:" << buzzer_timers.size());
  //     std::map<long, ros::Timer>::iterator itr;
  //     for (itr = light_timers.begin(); itr != light_timers.end(); ++itr) {
  //       ROS_INFO_STREAM('\t' << itr->first << '\n'); }
  //   }
  //   else
  //     return retval;
  // }
  return true;
}

bool::SrHazardLights::update_red_light(int pattern, int duration, bool reset)
{
  if (reset == true)
  {
      red_light_timers[default_key].buffer[4] = (red_light_timers[default_key].buffer[4] & 0xF0) | 0;
      red_light_ = false;
  }

  std::vector<uint8_t> buffer = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
  buffer[4] = (pattern << 4) | (buffer[4] & 0x0F);
  std::uint8_t* sent_buffer = &buffer[0];
  bool retval = send_buffer(sent_buffer);
  ROS_INFO_STREAM("red light buffer: \n");
  for (int i = 0; i < 8; i++)
  {
    ROS_INFO_STREAM(static_cast<int16_t>(sent_buffer[i]) << ", ");
  }
  ROS_INFO_STREAM("\n");
  if (!retval)
    return false;

  if (pattern != 0)
    red_light_ = true;
  else
    red_light_ = false;

  if (duration == 0)
  {
    red_light_timers[default_key].buffer = buffer;
  }
  else
  {
    ROS_ERROR("starting red light timer");
    ++timer_key;
    ROS_INFO_STREAM("red map size:" << red_light_timers.size());
    ROS_INFO_STREAM("red timer key:" << timer_key);
    ros::Timer light_timer = nh_.createTimer(ros::Duration(duration), std::bind(&SrHazardLights::light_timer_cb, this, timer_key, red_light_timers), true, true);
    light_timer.setPeriod(ros::Duration(duration), true);
    light_timer.start();
    red_light_timers.insert(std::pair<long,hazard_light_data>(timer_key, {light_timer, buffer}));
    ROS_INFO_STREAM("red map size:" << red_light_timers.size());
    ROS_INFO_STREAM("red map:");
    std::map<long, hazard_light_data>::iterator itr;
    for (itr = red_light_timers.begin(); itr != red_light_timers.end(); ++itr)
    {
      ROS_INFO_STREAM('\t' << itr->first << '\n');
    }
  }
  return true;
}

bool SrHazardLights::set_buzzer(int pattern, int tonea, int toneb, int duration, int reset)
{
  if (pattern < 0 || pattern > 9 || tonea > 15 || toneb > 15 || pattern < 0 || tonea < 0 || toneb < 0 || duration < 0)
  {
    ROS_ERROR("Number or duration chosen for buzzer is out of range");
    return false;
  }

  if (reset == true)
  {
    buzzer_timers[default_key].buffer[2] = 0x00;
    buzzer_timers[default_key].buffer[3] = 0x00;
    buzzer_on_ = false;
  }

  std::vector<uint8_t> buffer = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
  buffer[2] = (buffer[2] & 0xF0) | pattern;
  buffer[3] = (tonea << 4) + toneb;
  std::uint8_t* sent_buffer = &buffer[0];
  bool retval = send_buffer(sent_buffer);
  ROS_INFO_STREAM("red light buffer: \n");
  for (int i = 0; i < 8; i++)
  {
    ROS_INFO_STREAM(static_cast<int16_t>(sent_buffer[i]) << ", ");
  }
  ROS_INFO_STREAM("\n");
  if (!retval)
    return false;

  if (pattern != 0)
    buzzer_on_ = true;
  else
    buzzer_on_ = false;

  if (duration == 0)
  {
    buzzer_timers[default_key].buffer[2] = buffer[2];
    buzzer_timers[default_key].buffer[3] = buffer[3];
  }
  else
  {
    ROS_ERROR("starting buzzer timer");
    ROS_INFO_STREAM("map size:" << buzzer_timers.size());
    ++timer_key;
    ROS_INFO_STREAM("timer key:" << timer_key);
    ros::Timer buzzer_timer = nh_.createTimer(ros::Duration(duration), std::bind(&SrHazardLights::buzzer_timer_cb, this, timer_key), true, true);
    buzzer_timer.setPeriod(ros::Duration(duration), true);
    buzzer_timer.start();
    buzzer_timers.insert(std::pair<long, hazard_light_data>(timer_key, {buzzer_timer, buffer}));
    std::map<long, hazard_light_data>::iterator itr;
    ROS_INFO_STREAM("INTERATION: ");
    for (itr = buzzer_timers.begin(); itr != buzzer_timers.end(); ++itr) {
      ROS_INFO_STREAM('\t' << itr->first << '\n'); 
    }
    ROS_INFO_STREAM("map size:" << buzzer_timers.size());
  }
  return true;
}

bool SrHazardLights::send_buffer(std::uint8_t sent_buffer[8])
{
  int retval, rs = 0;
  retval = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT_OUT, sent_buffer, BUFFER_SIZE, &rs, TIMEOUT);
  if (retval)
  {
    ROS_ERROR("Hazard light failed to set with error: %s\n", libusb_error_name(retval));
    libusb_close(patlite_handle);
    patlite_handle = 0;
    return false;
  }
  return true;
}

void SrHazardLights::light_timer_cb(long timer_key_remove, std::map<long, hazard_light_data> light_timer_map)
{
  ROS_INFO_STREAM("map size COLOUR start of cb:" << light_timer_map.size());
  if (light_timer_map.size() < 3)
  {
    std::pair<long, hazard_light_data> reset_data = *(std::prev(light_timer_map.find(timer_key_remove)));

    ROS_INFO_STREAM("light return: " << reset_data.first << " and ");
    std::uint8_t* reset_buf = &reset_data.second.buffer[0];
    ROS_INFO_STREAM("light RESET buffer: \n");
    for (int i = 0; i < 8; i++)
    {
      ROS_INFO_STREAM(static_cast<int16_t>(reset_buf[i]) << ", ");
    }
    ROS_INFO_STREAM("\n");
    int retval, rs = 0;
    retval = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT_OUT, reset_buf, BUFFER_SIZE, &rs, 1000);
    ROS_ERROR("Here light 1");
    if (retval)
    {
      ROS_ERROR("Hazard light failed to set with error: %s\n", libusb_error_name(retval));
      libusb_close(patlite_handle);
      patlite_handle = 0;
    }
    // if ((reset_data.second.buffer[4] & 0xF0) == 0x00)
    //   red_light_ = false;
    // else if ((reset_data.second.buffer[4] & 0x0F) == 0x00)
    //   orange_light_ = false;
    // else if ((reset_data.second.buffer[5] & 0xF0) == 0x00)
    //   green_light_ = false;
    
    light_timer_map.erase(timer_key_remove);
  } else
      light_timer_map.erase(timer_key_remove);
  ROS_INFO_STREAM("map size COLOUR cb:" << light_timer_map.size());
  std::map<long, hazard_light_data>::iterator itr;
  ROS_INFO_STREAM("INTERATION COLOUR 222: ");
  for (itr = light_timer_map.begin(); itr != light_timer_map.end(); ++itr)
  {
    ROS_INFO_STREAM('\t' << itr->first << '\n'); 
  }
}

void SrHazardLights::buzzer_timer_cb(long timer_key_remove)
{
  ROS_INFO_STREAM("TIMER KEY REMOVE: " << timer_key_remove);
  if (buzzer_timers.size() < 3){
    std::pair<long, hazard_light_data> reset_data = *(std::prev(buzzer_timers.find(timer_key_remove)));
    ROS_INFO_STREAM("buzzer return: " << reset_data.first << " and ");
    std::uint8_t* reset_buf = &reset_data.second.buffer[0];
    ROS_INFO_STREAM("buzzer RESET buffer: \n");
    for (int i = 0; i < 8; i++)
    {
      ROS_INFO_STREAM(static_cast<int16_t>(reset_buf[i]) << ", ");
    }
    ROS_INFO_STREAM("\n");
    int retval, rs = 0;
    retval = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT_OUT, reset_buf, BUFFER_SIZE, &rs, 1000);
    ROS_ERROR("Here buzzer 1");
    if (retval)
    {
      ROS_ERROR("Hazard light failed to set with error: %s\n", libusb_error_name(retval));
      libusb_close(patlite_handle);
      patlite_handle = 0;
    }
    if ((reset_data.second.buffer[2] & 0x0F) == 0x00)
      buzzer_on_ = false;
    
    buzzer_timers.erase(timer_key_remove);
  } else
      buzzer_timers.erase(timer_key_remove);
  ROS_INFO_STREAM("map size cb:" << buzzer_timers.size());
  std::map<long, hazard_light_data>::iterator itr;
  ROS_INFO_STREAM("INTERATION 222: ");
  for (itr = buzzer_timers.begin(); itr != buzzer_timers.end(); ++itr) {
    ROS_INFO_STREAM('\t' << itr->first << '\n'); 
  }

}

// void SrHazardLights::buzzer_timer_cb(long timer_key_remove)
// {
//   int retval, rs = 0;
//   std::vector<uint8_t> reset_buffer = {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00};
//   reset_buffer[2] = default_buffer[2];
//   reset_buffer[3] = default_buffer[3];
//   std::uint8_t* reset_buf = &reset_buffer[0];
//   ROS_INFO_STREAM("buzzer RESET buffer: \n");
//   for (int i = 0; i < 8; i++)
//   {
//     ROS_INFO_STREAM(static_cast<int16_t>(reset_buf[i]) << ", ");
//   }
//   ROS_INFO_STREAM("\n");
//   retval = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT_OUT, reset_buf, BUFFER_SIZE, &rs, 1000);
//   ROS_ERROR("Here buzzer 1");
//   if (retval)
//   {
//     ROS_ERROR("Hazard light failed to set with error: %s\n", libusb_error_name(retval));
//     libusb_close(patlite_handle);
//     patlite_handle = 0;
//   }
//   if ((default_buffer[2] & 0x0F) == 0x00)
//     buzzer_on_ = false;

//   current_buffer[2] = default_buffer[2];
//   current_buffer[3] = default_buffer[3];
//   buzzer_timers.erase(timer_key_remove);
// }

bool SrHazardLights::reset_hazard_light(sr_hazard_light::ResetHazardLight::Request &request,
                                        sr_hazard_light::ResetHazardLight::Response &response)
{
  ROS_INFO("Resetting hazard light.");
  uint8_t reset_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  light_timers.clear();
  buzzer_timers.clear(); // need to change these to putting each 0 map to 0x00 etc etc;
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

void SrHazardLights::read_buffer()
{
  ROS_INFO("HELLO");
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
