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

#include <libusb-1.0/libusb.h>
#include "sr_hazard_light/sr_hazard_light_driver.h"

#define LIGHT_VID 0x191A
#define LIGHT_PID 0x8003
#define PATLITE_ENDPOINT 1

static libusb_device_handle *patlite_handle = 0;

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

SrHazardLights::SrHazardLights()
  :started_(false), context_(nullptr), connected_(false), detected_(false), 
  red_light_(false), orange_light_(false), green_light_(false), light_pattern_(0), 
  buzzer_on_(false), buzzer_type_(0)
{
  libusb_init(&context_);

  hazard_light_service = nh_.advertiseService("sr_hazard_light/set_hazard_light", &SrHazardLights::set_hazard_light, this);
}

SrHazardLights::~SrHazardLights()
{
  stop();
}

void SrHazardLights::start(int publishing_rate)
{
  publishing_rate_ = publishing_rate;
  buffer_ = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

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
  buffer_ = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::uint8_t* buf = &buffer_[0];
  SrHazardLights::set(0, buf);
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

bool SrHazardLights::open_device(){
  patlite_handle = libusb_open_device_with_vid_pid(NULL, LIGHT_VID, LIGHT_PID);
  if (patlite_handle == NULL) {
    ROS_ERROR("Hazard Light device not found\n");
    return false;
  }

  libusb_set_auto_detach_kernel_driver(patlite_handle, 1);

  int r;
  r = libusb_claim_interface(patlite_handle, 0);
  if (r != LIBUSB_SUCCESS) {
    ROS_ERROR("libusb claim failed\n");
    return false;
  }

  int rs=0;
  std::uint8_t* buf = &buffer_[0];
  r = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT, buf, sizeof(buf), &rs, 1000);

  if (r != 0) {
    ROS_ERROR("Hazard light reset failed, return %s , transferred %i\n", libusb_error_name(r), rs);
    libusb_close(patlite_handle);
    patlite_handle=0;
    return false;
  }

  connected_ = true;
  ROS_INFO("Hazard light USB device opened and claimed\n");

  return true;
};


void SrHazardLights::read_data_from_device(std::uint8_t* buffer)
{
  if (buffer[2] == buzzer_type_) {
    buzzer_on_ = true;
  } else {
    buzzer_on_ = false;
  }

  if (buffer[4] == light_pattern_<<4) {
    red_light_ = true;
  } else red_light_ = false;

  if (buffer[4] == light_pattern_) {
    orange_light_ = true;
  } else {
    orange_light_ = false;
  }
  
  if (buffer[5]= light_pattern_<<4) {
    green_light_ = true;
  } else {
    green_light_ = false;
  }
}


bool SrHazardLights::set_hazard_light(sr_hazard_light::SetLight::Request &request, 
                              sr_hazard_light::SetLight::Response &response) {
  light_pattern_ = request.light_pattern;
  std::string light_colour = request.light_colour;
  buzzer_type_ = request.buzzer_type;
  int buzzer_tonea = request.buzzer_tonea;
  int buzzer_toneb = request.buzzer_toneb;
  int duration = request.duration;
  bool reset = request.reset;

  if (reset == true) {
      buffer_ = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  }

  if (light_pattern_ > 9 || buzzer_type_ > 9 || buzzer_tonea > 15 || buzzer_toneb > 15)
    ROS_ERROR("no good");
    response.confirmation = false;

  std::list<std::string> light_colours = {"red", "orange", "green"}; // {"red", "orange", "green", "blue", "clear"};
  std::vector<uint8_t> changed_buffer_ = buffer_;
  if (light_pattern_ > 0) {
    if (std::find(std::begin(light_colours), std::end(light_colours), light_colour) != std::end(light_colours)) {
      if (light_colour == "red") {
        changed_buffer_[4] = light_pattern_<<4;
      }
      else if (light_colour == "orange") {
        changed_buffer_[4] = light_pattern_;
      }
      else if (light_colour == "green") {
        changed_buffer_[5] = light_pattern_<<4;
      }
      // Add this if we chose to buy these colours
      // else if (colour == "blue") {
      //   changed_buffer_[5] = pattern;
      // }
      // else if (colour == "clear") {
      //   changed_buffer_[6] = pattern<<4;
      // }
    }
    else {
      ROS_ERROR("Colour sent is not a colour on the hazard light");
      response.confirmation = false;
    }
  }

  if (buzzer_type_ > 0) {
      changed_buffer_[2] = buzzer_type_;
      changed_buffer_[3] = (buzzer_tonea<<4) + buzzer_toneb;
  }
  
  if (duration == 0) {
    buffer_ = changed_buffer_;
  }

  std::uint8_t* buf = &changed_buffer_[0];
  response.confirmation = SrHazardLights::set(duration, buf);
}

bool SrHazardLights::set(int duration, std::uint8_t buf[8]) {

  if (!patlite_handle) {
    if (!SrHazardLights::open_device()) {
      ROS_ERROR("Hazard light set failed, return false\n");
      return false;
    }
  }

  int r, rs=0;
  r = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT, buf, 8, &rs, 1000);
  read_data_from_device(buf);

  if (r) {
    ROS_ERROR("Hazard light failed to set with error: %s\n", libusb_error_name(r));    
    libusb_close(patlite_handle);
    patlite_handle=0;
    return false;
  }

  if (duration > 0) {
    ros::Duration(duration).sleep();
    std::uint8_t* reset_buf = &buffer_[0];
    r = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT, reset_buf, 8, &rs, 1000);
    read_data_from_device(reset_buf);
    if (r) {
      ROS_ERROR("Hazard light failed to set with error: %s\n", libusb_error_name(r));
      libusb_close(patlite_handle);
      patlite_handle=0;
      return false;
    }
  }

  return true;
}

void SrHazardLights::close_device()
{
  connected_ = false;
  red_light_ = false;
  orange_light_ = false;
  green_light_ = false;
  buzzer_on_ = false;
}

int SrHazardLights::on_usb_hotplug(struct libusb_context *ctx, struct libusb_device *device, libusb_hotplug_event event)
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