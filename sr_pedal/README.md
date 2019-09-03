# sr_pedal

This package allows the use of the "VEC USB Footpedal", a triple USB footpedal, as a ROS node. Launch with:

```bash
roslaunch sr_pedal sr_triple_pedal.py
```

And a topic will be published at `sr_pedal/status` with the format shown [here](msg/Status.msg). The topic is published at 20Hz by default; this can be changed with the `rate` argument. This default was chosen to allow heartbeat clients to use a 10Hz/0.1s timeout, ensuring a <0.1s response to pedal data being lost for whatever reason.

In order for this to work within docker, the following arguments must be added to your `docker run` command:

```bash
--device /dev/input -v /run/udev/data:/run/udev/data
```
