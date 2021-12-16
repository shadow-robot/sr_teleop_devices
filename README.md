# sr_piezo_feedback

## sr_piezo_feedback class

This class is used to transform tactile data received from the PST's sensors into output signal to provide haptic feedback.

```sh
roslaunch sr_piezo_feedback sr_piezo_feedback_launch.launch fingers:=<comma separated finger indexes> side:=<side>
```
Example

```sh
roslaunch sr_piezo_feedback sr_piezo_feedback_launch.launch fingers:=th,ff side:=rh
```

For testing purposes only, **pst_source.py** was added which will mock the data provided through the **/{side}/tactile** topic. The formulas/values there can be freely adjusted if required. To start the mock, start roscore and launch: 

```sh
python pst_source.py
```

## Prerequisites

As long as **sounddevice** is not available through rosdep, this library has to be installed manually with:

```sh
pip3 install sounddevice
```