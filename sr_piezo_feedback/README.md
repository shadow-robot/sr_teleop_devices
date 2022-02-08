# sr_piezo_feedback

## sr_piezo_feedback class

This class is used to transform tactile data received from the PST's sensors into output signal to provide haptic feedback.

```sh
roslaunch sr_piezo_feedback sr_piezo_feedback.launch fingers:=<comma separated finger indexes> hand_id:=<side>
```
Example

```sh
roslaunch sr_piezo_feedback sr_piezo_feedback.launch fingers:=th,ff hand_id:=rh
```

For testing purposes only, **pst_source.py** was added which will mock the data provided through the **/rh/tactile** topic. The formulas/values there can be freely adjusted if required. To start the mock, start roscore and launch: 

```sh
python pst_mock_source.py
```
