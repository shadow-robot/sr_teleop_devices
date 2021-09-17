# sr_teleop_manager

## Remote Power Control

This node interfaces with IP relays

Configuration information is stored in `config/power_devices.yaml`

To start the node:
```
rosrun sr_teleop_manager remote_power_control.py
```

Example relay request to set `register1` of the relay at `192.168.0.50` to `1`:

```
rosservice call /custom_power_relay_command "{ip_address: '192.168.0.50', command_string: 'state.xml?register1=1'}"
```

