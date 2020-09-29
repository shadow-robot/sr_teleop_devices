# sr_teleop_manager

## Remote Power Control

This node interfaces with IP relays for remote power on/off of a UR arm or shadow hand.

Configuration information is stored in `config/power_devices.yaml`

To start the node:
```bash
rosrun sr_utilities_common remote_power_control.py
```

Then pass a list of device names (as specified in `power_devices.yaml`) to `power_on` and/or `power_off` of the `/remote_power_control/goal` topic. E.g, to power the `right_arm` and `left_arm` on and the `right_hand` off, do:

```bash
rostopic pub /remote_power_control/goal sr_utilities_common/PowerManagerActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  power_on:
  - 'left_arm'
  - 'right_arm'
  power_off:
  - 'right_hand'" 

```

Progress will be posted to `/remote_power_control/feedback` and the final `success/fail` is published to `/remote_power_control/result`

