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
  :started_(false), context_(nullptr), connected_(false), detected_(true)
  // , light_pattern_(1), 
  // light_colours_([1, 1, 1, 1, 1]), buzzer_pattern_(0),  buzzer_tone_a_(0), buzzer_tone_b_(0)
{
  libusb_init(&context_);
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

    start separate thread to detect hotplug events
    hotplug_loop_thread_ = std::thread(&SrHazardLights::hotplug_loop, this);

    while (ros::ok() && started_)
    {
      if (detected_)
      {
        if (!connected_)
          open_device();
        // read_data_from_device();
      }
      else
        close_device();

      // publish_pedal_data();
      ros::Rate(publishing_rate_).sleep();
      ros::spinOnce();
    }
  }
}

void SrHazardLights::stop()
{
  libusb_hotplug_deregister_callback(context_, hotplug_callback_handle_);
  // hid_close(device_handle_);
  // hid_exit();
  started_ = false;
  hotplug_loop_thread_.join();
  libusb_exit(NULL);
  ROS_INFO("EXITING");
  exit(0);
}


int SrHazardLights::open_device(){
  int r;
  r = libusb_init(NULL);
  if (r < 0) {
    std::cout << "libusb init failed, err " << r << "\n" << std::endl;
    return r;
  }

  patlite_handle = libusb_open_device_with_vid_pid(NULL, PATLITE_VID, PATLITE_PID);
  if (patlite_handle == NULL) {
    ROS_ERROR("Hazard Light device not found\n");
    return -1;
  }

  std::cout << patlite_handle << std::endl;

  libusb_set_auto_detach_kernel_driver(patlite_handle, 1);

  r = libusb_claim_interface(patlite_handle, 0);
  if (r != LIBUSB_SUCCESS) {
    ROS_ERROR("libusb claim failed\n");
    return -1;
  }

  std::uint8_t patlite_buf[8] = {0x00, 0x00, 0x08, 0xff, 0x00, 0x00, 0x00, 0x00};

  int rs=0;
  r = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT, patlite_buf, sizeof(patlite_buf), &rs, 1000);

  if (r != 0) {
    std::cout << "Patlite reset failed, return " << r << ", transferred " << rs << "\n" << std::endl;
    libusb_close(patlite_handle);
    patlite_handle=0;
    return -1;
  }

  ROS_INFO("Patlite USB device opened and claimed\n");

  return 0;
};

// int SrHazardLights::patlite_lights(int red, int yellow, int green, int blue, int clear) {
//   std::uint8_t buf[8] = {0x00, 0x00, 0x08, 0xff, (red<<4) + yellow, (green<<4) + blue, (clear<<4), 0x00};

//   if (red > 9 || yellow > 9 || green > 9 || blue > 9 || clear > 9)
//     return 1;
  
//   return SrHazardLights::patlite_set(buf);
// }

// int SrHazardLights::patlite_buzzer(int type, int tonea, int toneb) {
//   std::uint8_t buf[8] = {0x00, 0x00, type, (tonea<<4) + toneb, 0x88, 0x88, 0x80, 0x00};

//   if (tonea > 15 || toneb > 15)
//     return 1;
  
//   return SrHazardLights::patlite_set(buf);
// }

// int SrHazardLights::patlite_set(uint8_t *buf) {
//   int r;

//   //Do we have valid handle?
//   if (!patlite_handle) {
//     r = SrHazardLights::open_device();
//     if (r)
//       return r;
//   }

//   int rs=0;
//   r = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT, buf, 8, &rs, 1000);

//   if (r) {
//     std::cout << "Patlite set failed, return " << r << "\n" << std::endl;
//     libusb_close(patlite_handle);
//     patlite_handle=0;
//     return 2;
//   }

//   return 0;
// }

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
