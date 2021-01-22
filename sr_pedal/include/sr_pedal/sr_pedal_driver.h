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

#ifndef SR_PEDAL_DRIVER_H
#define SR_PEDAL_DRIVER_H

#include <ros/ros.h>
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <thread>  // NOLINT(build/c++11)
#include <hidapi/hidapi.h>
#include <sr_pedal/Status.h>
#include <atomic>

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
    bool left_pressed_;
    bool right_pressed_;
    bool middle_pressed_;
    bool connected_;
    std::atomic<bool> detected_;
    sr_pedal::Status sr_pedal_status_;
    int publishing_rate_;
    ros::Publisher pedal_publisher_ = nh_.advertise<sr_pedal::Status>("sr_pedal/status", 1);
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

#endif  //  SR_PEDAL_DRIVER_H
