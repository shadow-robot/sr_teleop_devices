/*
* Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef SR_TRIPLE_PEDAL_HPP
#define SR_TRIPLE_PEDAL_HPP

#include <ros/ros.h>
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <thread>  // NOLINT(build/c++11)
#include <hidapi/hidapi.h>
#include <sr_pedal/Status.h>
#include <atomic>

#define PEDAL_VENDOR 0x05f3
#define PEDAL_ID 0x00ff
#define RAW_LEFT_BUTTON_PRESSED_VALUE 49
#define RAW_MIDDLE_BUTTON_PRESSED_VALUE 50
#define RAW_MID_LEFT_BUTTON_PRESSED_VALUE 51
#define RAW_RIGHT_BUTTON_PRESSED_VALUE 52
#define RAW_LEFT_RIGHT_BUTTON_PRESSED_VALUE 53
#define RAW_MID_RIGHT_BUTTON_PRESSED_VALUE 54
#define RAW_ALL_PRESSED_VALUE 55

class SrTriplePedal
{
  public:
    SrTriplePedal();
    ~SrTriplePedal();

    void start(int publishing_rate);
    void stop();

  private:
    ros::NodeHandle nh_ = ros::NodeHandle();
    hid_device *device_handle;
    bool started_;
    libusb_context *context_;
    libusb_hotplug_callback_handle hotplug_callback_handle_;
    std::thread hotplug_loop_thread_;
    std::thread run_thread_;
    bool left_pressed_;
    bool right_pressed_;
    bool middle_pressed_;
    bool connected_;
    std::atomic<bool> detected_;
    sr_pedal::Status sr_pedal_status_;
    int publishing_rate_;
    ros::Publisher pedal_publisher_ = nh_.advertise<sr_pedal::Status>("sr_pedal/status", 1);
    char data_[8];
    unsigned char buffer_[8];
    void open_device();
    void read_data_from_device();
    void close_device();
    void detect_device_event(libusb_hotplug_event event);
    int on_usb_hotplug(struct libusb_context *ctx,
                        struct libusb_device *device,
                        libusb_hotplug_event event);
    static int on_usb_hotplug_callback(struct libusb_context *ctx,
                                 struct libusb_device *device,
                                 libusb_hotplug_event event,
                                 void* discovery);
    void hotplug_loop();
    void publish_pedal_data();
    void map_command_received(int raw_data_received);
};

#endif  //  SR_TRIPLE_PEDAL_HPP
