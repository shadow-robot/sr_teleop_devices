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

#include "sr_pedal/sr_pedal_driver.h"
#include <typeinfo>

#define PEDAL_VENDOR 0x05f3
#define PEDAL_ID 0x00ff
#define RAW_LEFT_BUTTON_PRESSED_VALUE 1
#define RAW_MIDDLE_BUTTON_PRESSED_VALUE 2
#define RAW_MID_LEFT_BUTTON_PRESSED_VALUE 3
#define RAW_RIGHT_BUTTON_PRESSED_VALUE 4
#define RAW_LEFT_RIGHT_BUTTON_PRESSED_VALUE 5
#define RAW_MID_RIGHT_BUTTON_PRESSED_VALUE 6
#define RAW_ALL_PRESSED_VALUE 7

SrTriplePedal::SrTriplePedal()
  :started_(false), context_(nullptr), right_pressed_(false), left_pressed_(false),
  middle_pressed_(false), connected_(false), detected_(false)
{
  libusb_init(&context_);
}

SrTriplePedal::~SrTriplePedal()
{
  stop();
}

void SrTriplePedal::start(int publishing_rate)
{
  publishing_rate_ = publishing_rate;
  if (!started_)
  {
    int ret;
    ret = libusb_hotplug_register_callback(context_,
            (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
            LIBUSB_HOTPLUG_ENUMERATE, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
            &SrTriplePedal::on_usb_hotplug_callback, reinterpret_cast<void*>(this), &hotplug_callback_handle_);

    if (LIBUSB_SUCCESS == ret)
      started_ = true;

    // start separate thread to detect hotplug events
    hotplug_loop_thread_ = std::thread(&SrTriplePedal::hotplug_loop, this);

    while (ros::ok() && started_)
    {
      if (detected_)
      {
        if (!connected_)
          open_device();
        read_data_from_device();
      }
      else
        close_device();

      publish_pedal_data();
      ros::Rate(publishing_rate_).sleep();
      ros::spinOnce();
    }
  }
}

void SrTriplePedal::stop()
{
  libusb_hotplug_deregister_callback(context_, hotplug_callback_handle_);
  hid_close(device_handle);
  hid_exit();
  started_ = false;
  hotplug_loop_thread_.join();
  libusb_exit(context_);
}

void SrTriplePedal::detect_device_event(libusb_hotplug_event event)
{
  switch (event)
  {
    case LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED:
      ROS_INFO("Pedal device detected");
      detected_ = true;
      break;
    case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT:
      ROS_INFO("Pedal disconnected");
      detected_ = false;
      break;
    default:
      ROS_WARN("Unknown event detected");
      break;
  }
}

void SrTriplePedal::open_device()
{
  device_handle = hid_open(PEDAL_VENDOR, PEDAL_ID, NULL);
  if (device_handle)
  {
    ROS_INFO("Pedal device connected");
    connected_ = true;
    hid_set_nonblocking(device_handle, 1);
  }
  else
  {
    ROS_ERROR("Could not open pedal");
  }
}

void SrTriplePedal::read_data_from_device()
{
  int raw_data_received = 0;
  int res = hid_read(device_handle, buffer_, sizeof(buffer_));

  if (res < 0)
  {
    ROS_WARN("Unable to read data from pedal");
  }
  else
  {
    raw_data_received = static_cast<int>(buffer_[0]);
    left_pressed_ = false;
    middle_pressed_ = false;
    right_pressed_ = false;

    map_command_received(raw_data_received);
  }
}

void SrTriplePedal::map_command_received(int raw_data_received)
{
  switch (raw_data_received)
  {
    case RAW_LEFT_BUTTON_PRESSED_VALUE:
      left_pressed_ = true;
      break;
    case RAW_MIDDLE_BUTTON_PRESSED_VALUE:
      middle_pressed_ = true;
      break;
    case RAW_RIGHT_BUTTON_PRESSED_VALUE:
      right_pressed_ = true;
      break;
    case RAW_LEFT_RIGHT_BUTTON_PRESSED_VALUE:
      left_pressed_ = true;
      right_pressed_ = true;
      break;
    case RAW_ALL_PRESSED_VALUE:
      left_pressed_ = true;
      right_pressed_ = true;
      middle_pressed_ = true;
      break;
    case RAW_MID_RIGHT_BUTTON_PRESSED_VALUE:
      right_pressed_ = true;
      middle_pressed_ = true;
      break;
    case RAW_MID_LEFT_BUTTON_PRESSED_VALUE:
      left_pressed_ = true;
      middle_pressed_ = true;
      break;
  }
}

void SrTriplePedal::close_device()
{
  connected_ = false;
  left_pressed_ = false;
  middle_pressed_ = false;
  right_pressed_ = false;
}

int SrTriplePedal::on_usb_hotplug(struct libusb_context *ctx, struct libusb_device *device, libusb_hotplug_event event)
{
  struct libusb_device_descriptor descriptor;

  int ret = libusb_get_device_descriptor(device, &descriptor);
  if (PEDAL_VENDOR == descriptor.idVendor && PEDAL_ID == descriptor.idProduct)
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


int SrTriplePedal::on_usb_hotplug_callback(struct libusb_context *ctx,
                                           struct libusb_device *device,
                                           libusb_hotplug_event event,
                                           void* pedal_discovery)
{
  return (reinterpret_cast<SrTriplePedal*>(pedal_discovery))->on_usb_hotplug(ctx, device, event);
}

void SrTriplePedal::hotplug_loop()
{
  while (started_)
  {
    libusb_handle_events_completed(context_, nullptr);
    ros::Rate(publishing_rate_).sleep();
  }
}

void SrTriplePedal::publish_pedal_data()
{
  sr_pedal_status_.header.stamp = ros::Time::now();
  sr_pedal_status_.connected = connected_;
  sr_pedal_status_.left_pressed = left_pressed_;
  sr_pedal_status_.middle_pressed = middle_pressed_;
  sr_pedal_status_.right_pressed = right_pressed_;
  pedal_publisher_.publish(sr_pedal_status_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sr_teleop_pedal");
    SrTriplePedal sr_triple_pedal;

    int publishing_rate;
    ros::param::param<int>("~publishing_rate", publishing_rate, 20);

    sr_triple_pedal.start(publishing_rate);
    ros::spin();

    return 0;
}
