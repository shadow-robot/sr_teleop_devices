/*
* Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include "sr_pedal_driver.hpp"
#include <typeinfo>

SrTriplePedal::SrTriplePedal()
  :started_(false), context_(nullptr), right_pressed_(false), left_pressed_(false),
  middle_pressed_(false), connected_(false)
{
  libusb_init(&context_);
}

SrTriplePedal::~SrTriplePedal()
{
  stop();
}

void SrTriplePedal::start()
{
  if (!started_)
  {
    int ret;
    ret = libusb_hotplug_register_callback(context_,
            (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
            LIBUSB_HOTPLUG_ENUMERATE, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
            &SrTriplePedal::on_usb_hotplug_callback, reinterpret_cast<void*>(this), &hotplug_callback_handle_);

    if (LIBUSB_SUCCESS == ret)
      started_ = true;

    hotplug_loop_thread_ = std::thread(&SrTriplePedal::hotplug_loop, this);
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
      ROS_INFO("Pedal detected");
      open_device();
      break;
    case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT:
      ROS_INFO("Pedal disconnected");
      close_device();
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
    ROS_INFO("Pedal device connected correctly");
    connected_ = true;
    hid_set_nonblocking(device_handle, 1);
    read_data_from_device();
  }
  else
  {
    ROS_ERROR("Could not open pedal");
  }
}

void SrTriplePedal::read_data_from_device()
{
  char data[8];
  unsigned char buffer[8];
  int res = 0;
  int raw_data_received = 0;
  while (res >= 0 && ros::ok())
  {
    res = hid_read(device_handle, buffer, sizeof(buffer));

    if (res < 0)
      ROS_WARN("Unable to read data from pedal");
    else
    {
      snprintf(data, sizeof(data), "%02x %02x", buffer[0], buffer[1]);

      raw_data_received = data[1];  // the second char is the one that changes when buttons are pressed
      left_pressed_ = false;
      middle_pressed_ = false;
      right_pressed_ = false;

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
      }
    }
    publish_pedal_data();
    publish_rate_.sleep();
    ros::spinOnce();
  }
}

void SrTriplePedal::close_device()
{
  connected_ = false;
  left_pressed_ = false;
  middle_pressed_ = false;
  right_pressed_ = false;
  publish_pedal_data();
  hid_close(device_handle);
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
    publish_rate_.sleep();
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

    sr_triple_pedal.start();
    ros::spin();

    return 0;
}
