# sr_pedal

This package allows the use of the [VEC Infinity USB Footpedal](https://www.amazon.co.uk/INFINITY-USB-FOOT-PEDAL-IN-USB-2/dp/B002MY6I7G), a triple USB footpedal, as a ROS node on Windows. Launch with:

```bash
roslaunch sr_pedal sr_pedal.launch
```
By default, the node will look for the roscore at remote 'ros_master_uri=http://10.9.11.1:11311"' with 'ros_ip="10.9.12.1"'. If you wish to run it locally, change these parameters to the IP of your machine. 

When launched a topic will be published at `sr_pedal/status` with the format shown [here](msg/Status.msg). The topic is published at 20Hz by default; this can be changed with the `rate` argument. This default was chosen to allow heartbeat clients to use a 10Hz/0.1s timeout, ensuring a <0.1s response to pedal data being lost for whatever reason.
