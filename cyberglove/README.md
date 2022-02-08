**cyberglove** is a generic ROS interface to Immersion's Cyberglove dataglove. It reads the data from the Cyberglove, calibrate them using a calibration file and stream them to 2 different `/joint_states` topics: one for the raw data the other one for the calibrated data. There's a utility in `sr_control_gui` which can be used to generate a calibration file for a specific user in a few steps.

If the button on the wrist is off, the glove won't publish any data.

The calibration file can't be dynamically loaded for the time being, so if you change the calibration then don't forget to restart the cyberglove node.

How To Use
----------

To run the cyberglove node, just run:

```
$ roslaunch cyberglove cyberglove.launch
```

You can specify some parameters in the launch file:

* cyberglove_prefix The prefix to put in front of the joint_states published by the glove.
* publish_frequency The frequency at which you want to publish the data.
* path_to_glove The path to the port on which the Cyberglove is connected (usually `/dev/ttyS0`)
* path_to_calibration The path to the calibration file for the Cyberglove

Code API
--------

* serial_glove.h The C interface to interact with the cyberglove
* xml_calibration_parser::XmlCalibrationParser The calibration file parser.
* cyberglove_service::CybergloveService A service which can stop / start the Cyberglove publisher.
* cyberglove_publisher::CyberglovePublisher The actual publisher streaming the data from the cyberglove.
