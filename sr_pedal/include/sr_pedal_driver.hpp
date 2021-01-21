
#ifndef SR_TRIPLE_PEDAL_HPP
#define SR_TRIPLE_PEDAL_HPP

#include <ros/ros.h>
#include <libusb-1.0/libusb.h>
#include <iomanip>
#include <iostream>
#include <thread>
#include <hidapi/hidapi.h>
#include <sr_pedal/Status.h>

#define PEDAL_VENDOR 0x05f3
#define PEDAL_ID 0x00ff

class SrTriplePedal
{
  public:
    SrTriplePedal();
    ~SrTriplePedal();
    
    void start();
    void stop();
    void device_open();
    void device_read();
    void device_close();

    void detect_device_event(struct libusb_device *device, int16_t vid, int16_t pid, libusb_hotplug_event event);

    hid_device *handle;
    uint32_t benchPackets;
    uint32_t benchBytes;
    struct timespec t1, t2;
    uint32_t diff;
  private:
    ros::NodeHandle _nh = ros::NodeHandle();
    bool _started;
    libusb_context *_context;
    libusb_hotplug_callback_handle _handle;
    std::shared_ptr<std::thread> _loop_thread;
    bool left_pressed;
    bool right_pressed;
    bool middle_pressed;
    bool connected;
    sr_pedal::Status sr_pedal_status_;

    ros::Rate publish_rate_ = ros::Rate(125);
    int _on_usb_hotplug(struct libusb_context *ctx,
                        struct libusb_device *device,
                        libusb_hotplug_event event);
    static int _on_usb_hotplug_callback(struct libusb_context *ctx,
                                 struct libusb_device *device,
                                 libusb_hotplug_event event,
                                 void* discovery);
    void _on_data_in(struct libusb_transfer *transfer);

    static void _on_data_in_callback(struct libusb_transfer *transfer, void* pedal_discovery);
    void _loop();
    ros::Publisher _pedal_publisher = _nh.advertise<sr_pedal::Status>("sr_pedal/status", 1);
    void _publish_pedal_data();
};

#endif  //  SR_TRIPLE_PEDAL_HPP

