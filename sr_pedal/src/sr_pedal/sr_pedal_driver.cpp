#include "sr_pedal_driver.hpp"
#include <typeinfo>

SrTriplePedal::SrTriplePedal()
  :_started(false), _context(nullptr), right_pressed(false), left_pressed(false), middle_pressed(false), connected(false)
{
  libusb_init(&_context);
}

SrTriplePedal::~SrTriplePedal()
{
  stop();
  libusb_exit(_context);
}

void SrTriplePedal::start()
{

  if (!_started)
  {
    int ret;
    ret = libusb_hotplug_register_callback(_context,
                                           (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
                                           LIBUSB_HOTPLUG_ENUMERATE,
                                           LIBUSB_HOTPLUG_MATCH_ANY, // vid
                                           LIBUSB_HOTPLUG_MATCH_ANY, // pid
                                           LIBUSB_HOTPLUG_MATCH_ANY, // class
                                           &SrTriplePedal::_on_usb_hotplug_callback,
                                           (void*)this,
                                           &_handle);
    if (LIBUSB_SUCCESS == ret)
    {
      _started = true;
    }
    _loop_thread = new std::thread([this] { this->_loop(); });
   }
}

void SrTriplePedal::stop()
{
  if (_started)
  {
    libusb_hotplug_deregister_callback(_context, _handle);
    _started = false;
    hid_exit();

    _loop_thread->join();
    delete _loop_thread;
    _loop_thread = nullptr;
  }
}

void SrTriplePedal::detect_device_event(struct libusb_device *device, int16_t vid, int16_t pid, libusb_hotplug_event event)
{
  std::string event_string;
 
  switch (event)
  {
    case LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED:
      ROS_INFO("Pedal detected");
      device_open();
      break;
    case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT:
      ROS_INFO("Pedal disconnected");
      device_close();
      break;
    default:
      event_string = "unknown";
      break;
  }
}

void SrTriplePedal::device_open()
{
  handle = hid_open(PEDAL_VENDOR, PEDAL_ID, NULL);
  if (!handle)
  {
    ROS_ERROR("Could not open pedal");
  }
  else
  {
    ROS_INFO("Pedal device connected correctly");
    connected = true;
    hid_set_nonblocking(handle, 1);
    device_read();
  }
}

void SrTriplePedal::device_read()
{
  char data[256];
  unsigned char buf[256];
  int res = 0;
  while (res >= 0 && ros::ok()) {
    res = hid_read(handle, buf, sizeof(buf));

    if (res < 0)
      ROS_WARN("Unable to read data from pedal\n");
    else
    {
      sprintf(data, "%02x %02x %02x %02x -- %02x %02x %02x %02x -- %02x %02x %02x %02x", 
		          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
              buf[7], buf[8], buf[9], buf[10], buf[11]);
      //std::cout<<"data: "<<data<<"\n";
      left_pressed = false;
      middle_pressed = false;
      right_pressed = false;

      if ((data[1] & 0x1) != 0)
      {
        left_pressed = true;
      }
      if ((data[1] & 0x2) != 0)
      {
        middle_pressed = true;
      }
      if ((data[1] & 0x4) != 0)
      {
        right_pressed = true;
      }
    }
    _publish_pedal_data();
    publish_rate_.sleep();
    ros::spinOnce();
  }
}

void SrTriplePedal::device_close()
{
  connected = false;
  left_pressed = false;
  middle_pressed = false;
  right_pressed = false;
  _publish_pedal_data();
  hid_close(handle);
}

int SrTriplePedal::_on_usb_hotplug(struct libusb_context *ctx, struct libusb_device *device, libusb_hotplug_event event)
{
  struct libusb_device_descriptor descriptor;

  int ret = libusb_get_device_descriptor(device, &descriptor);
  if (PEDAL_VENDOR == descriptor.idVendor && PEDAL_ID == descriptor.idProduct)
  {
    if (LIBUSB_SUCCESS == ret)
    {
      detect_device_event(device, descriptor.idVendor, descriptor.idProduct, event);
    }
    else
    {
      std::cout << "Got a device but failed to look up its descriptor" << std::endl;
    }
  }
  return 0;
}


int SrTriplePedal::_on_usb_hotplug_callback(struct libusb_context *ctx,
                                                   struct libusb_device *device,
                                                   libusb_hotplug_event event,
                                                    void* pedal_discovery)
{
  return ((SrTriplePedal*)pedal_discovery)->_on_usb_hotplug(ctx, device, event);
}

void SrTriplePedal::_loop()
{
  while (_started)
  {
    libusb_handle_events_completed(_context, nullptr);
  }
}

void SrTriplePedal::_publish_pedal_data()
{
  sr_pedal_status_.connected = connected;
  sr_pedal_status_.left_pressed = left_pressed;
  sr_pedal_status_.middle_pressed = middle_pressed;
  sr_pedal_status_.right_pressed = right_pressed;
  _pedal_publisher.publish(sr_pedal_status_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sr_teleop_pedal");
    SrTriplePedal sr_triple_pedal;

    sr_triple_pedal.start();
    ros::spin();

    return 0;
}
