#include <ros/ros.h>
#include <libusb-1.0/libusb.h>
#include <sr_pedal/Status.h>
#include <iomanip>
#include <iostream>
#include <thread>

class Discovery
{
public:
    Discovery()
    : started(false),
    context(nullptr)
    {
      libusb_init(&context);
    }

    ~Discovery()
    {
      Stop();
      libusb_exit(context);
    }

    void Start()
    {
      std::cout << "Start" << std::endl;
      if (!started)
      {
        int ret;
        ret = libusb_hotplug_register_callback(context,
                                               (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
                                                LIBUSB_HOTPLUG_ENUMERATE,
                                                LIBUSB_HOTPLUG_MATCH_ANY, // vid
                                                LIBUSB_HOTPLUG_MATCH_ANY, // pid
                                                LIBUSB_HOTPLUG_MATCH_ANY, // class
                                                OnUsbHotplugCallback,
                                                (void*)this,
                                                &handle);

        if (LIBUSB_SUCCESS == ret)
        {
          started = true;
        }
        loop = new std::thread([this] { this->Loop(); });
      }
  }

    void Stop()
    {
        if (started)
        {
            std::cout << "Stop" << std::endl;
            libusb_hotplug_deregister_callback(context, handle);
            started = false;

            loop->join();
            delete loop;
            loop = nullptr;
        }
    }

    static void LogEvent(int16_t vid, int16_t pid, libusb_hotplug_event event)
    {
        std::string event_string;

        switch (event)
        {
            case LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED:
                event_string = "arrived";
                break;
            case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT:
                event_string = "left";
                break;
            default:
                event_string = "unknown";
                break;
        }
        std::cout << "Got a device: " << std::hex << vid << "," << pid << "," << event_string << std::dec << std::endl;
    }

private:
    int OnUsbHotplug(struct libusb_context *ctx,
                     struct libusb_device *device,
                     libusb_hotplug_event event)
    {
        struct libusb_device_descriptor descriptor;

        int ret = libusb_get_device_descriptor(device, &descriptor);
        if (LIBUSB_SUCCESS == ret)
        {
            LogEvent(descriptor.idVendor, descriptor.idProduct, event);
        }
        else
        {
            std::cout << "Got a device but failed to look up its descriptor" << std::endl;
        }

        return 0;
    }

    static int OnUsbHotplugCallback(struct libusb_context *ctx,
                                    struct libusb_device *device,
                                    libusb_hotplug_event event,
                                    void* discovery)
    {
        return ((Discovery*)discovery)->OnUsbHotplug(ctx, device, event);
    }

    void Loop()
    {
        while (started)
        {
            libusb_handle_events_completed(context, nullptr);
        }
    }

    bool started;
    libusb_context *context;
    libusb_hotplug_callback_handle handle;

    std::thread *loop;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sr_teleop_pedal");
    std::cout << "Press return to quit..." << std::endl;

    Discovery discovery;
    discovery.Start();

    std::cin.get();

    discovery.Stop();

    return 0;
}
