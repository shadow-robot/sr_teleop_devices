# sr_hazard_light

This package allows the use of [Patlite LR6-USB USB Signal Tower](https://www.patlite.com/product/detail0000000689.html), a USB Controlled Modular Signal Light Tower, as a ROS Node.

Launch with:
```bash
roslaunch sr_hazard_light sr_hazard_light.launch
```

A topic will be published at `rostopic echo /sr_hazard_light/status` with the format shown [here](msg/Status.msg). The topic is published at 1Hz by default; this can be changed with the `rate` argument.

## /sr_hazard_light/set_hazard_light service

A service will be advertised called `/sr_hazard_light/set_hazard_light` with the format shown [here](srv/SetHazardLight.srv).

This service can be called on the terminal by: 

```bash 
rosservice call /sr_hazard_light/set_hazard_light "light:
- pattern: 0                                             
  colour: ''                                           
  duration: 0
buzzer:
- pattern: 0
  tonea: 0                                         
  toneb: 0
  duration: 0 "
```
where:

| Variable | Details | Values |
| :-------------:|:-------------:|:-------------:|
| `light: pattern`| sets the LED control values for LED control of R/O/G | 0 off, 1-5 patterns |
| `light: colour` | sets the LED colour | red, orange or green         |
| `buzzer: pattern`  | sets the buzzer pattern  | 0 off, 1-5 patterns |
| `buzzer: tonea` | sets the pitch values for Sound A | 0 off, 1-13 pitches |
| `buzzer: toneb` | sets the pitch values for Sound B | 0 off, 1-13 pitches |
| `light/buzzer: duration`     | sets the duration of the combination (seconds) | 0 ongoing, 1< second duration  |

You can also send partial/multiple services:
```bash
rosservice call /sr_hazard_light/set_hazard_light "light:
- pattern: 0, colour: '', duration: 0"

rosservice call /sr_hazard_light/set_hazard_light "buzzer:
- pattern: 0, tonea: 0, toneb: 0, duration: 0"

rosservice call /sr_hazard_light/set_hazard_light "light:
- {pattern: 0, colour: '', duration: 0}
- {pattern: 0, colour: '', duration: 0}"
```

The service can also be called within a node using a ROS Service Client in [Python](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29) and [Cpp](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29).

## /sr_hazard_light/reset_hazard_light service

A service will be advertised called `/sr_hazard_light/reset_hazard_light` which will reset the hazard light.

This service can be called on the terminal by: 

```bash
rosservice call /sr_hazard_light/reset_hazard_light 
```

The service can also be called within a node using a ROS Service Client in [Python](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29) and [Cpp](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29).

## Udev Rules

Udev rules are required to allow any user to read data from the pedal device. Place [90-hazard-light.rules](90-hazard-light.rules) in `/etc/udev/rules.d` and run:

```bash
sudo udevadm control --reload && sudo udevadm trigger
```
to reload the rules.

In order for this to work within docker, the following arguments must be added to your `docker run` command:

```bash
-v /dev/dev -v /run/udev/data:/run/udev/data
```