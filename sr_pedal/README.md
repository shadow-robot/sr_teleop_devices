## sr_pedal

# Runing 

This package allows the use of the [VEC Infinity USB Footpedal](https://www.amazon.co.uk/INFINITY-USB-FOOT-PEDAL-IN-USB-2/dp/B002MY6I7G), a triple USB footpedal, as a ROS node on Windows. Launch with:

```bash
roslaunch sr_pedal sr_pedal.launch
```
By default, the node will look for the roscore at remote `ros_master_uri="http://10.9.11.1:11311"` with `ros_ip=10.9.12.1` If you wish to run it locally, change these parameters to the IP of your machine. 

When launched a topic will be published at `sr_pedal/status` with the format shown [here](msg/Status.msg). The topic is published at 20Hz by default; this can be changed with the `rate` argument. This default was chosen to allow heartbeat clients to use a 10Hz/0.1s timeout, ensuring a <0.1s response to pedal data being lost for whatever reason.


# Building

If ROS is not installed on the Windows machine follow the steps mentioned here

 [Installation/Windows - ROS Wiki](http://wiki.ros.org/Installation/Windows)

As the sr_pedal_node requires libusb and hidapi librarires you need to install them via [vcpkg](https://github.com/microsoft/vcpkg) as they are not available for [chocolatey](https://github.com/ms-iot/rosdistro-db/blob/init_windows/rosdep/win-chocolatey.yaml). To do so, update first the ports (inside C:\opt\ros\noetic\x64\tools\vcpkg) folder via manually replacing it or via git pull. Next run .\bootstrap-vcpkg.bat  and vcpkg update.

Now you should be able to install the libraries via:
```bash
vcpkg install libusb 
vcpkg install hidapi
```

The differences between Ubuntu and Windows version are: 

- CMakeLists.txt - for some reason catkin_make can’t find the vcpkg libraries so here the include_directories and target_link_libraries are having the directs paths listed.

- sr_pedal_driver.cpp - removed code responsible for hotplugging as it is not supported on Windows. Therefore whenever you unplugg the pedal, you have to restart the node.

Build the package and source the workspace by running the setup.bat file within the devel folder of the package. 

