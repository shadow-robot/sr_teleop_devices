# sr_leap_motion

This package contains an interface between Shadow software and Leap motion software. It relies on the leap_motion package.

## Prerequisites

The Leap Motion SDK V2 for Linux is required; you can get it [here](https://developer.leapmotion.com/setup/desktop). Download the SDK; the following instructions assume to your `~/Downloads` directory.

```bash
cd Downloads && tar -zxvf Leap_Motion_SDK_Linux_2.3.1.tgz && \
sudo dpkg -i LeapDeveloperKit_2.3.1+31549_linux/Leap-2.3.1+31549-x64.deb && \
sudo apt install -f && \
rm -rf LeapDeveloperKit_2.3.1+31549_linux && rm Leap_Motion_SDK_Linux_2.3.1.tgz
```

We need the Leap service to be running all the time; Ubuntu has changed how services work since V2 was released. As a result, you should copy `leapd.service` into systemd's folders and enable the service:

```bash
sudo cp $(rospack find sr_leap_motion)/resources/leapd.service lib/systemd/system/. && \
sudo ln -s /lib/systemd/system/leapd.service /etc/systemd/system/leapd.service && \
sudo systemctl daemon-reload && \
sudo service leapd start
```

There are also the usual ROS prerequisite steps (rosdep to install binary dependencies listed in [package.xml](package.xml), wstool and catkin to fetch and install src dependencies).

## Running

### Stand-alone

`sr_leap_motion.launch` is the main launchfile - it will start Leap nodes and publish user hand TFs and a heartbeat topic. You can run it stand-alone and visualise the data by running:

```bash
roslaunch sr_leap_motion sr_leap_motion.launch rviz:=true
```

### Controlling a Shadow Dexterous Hand

Currently only a right Dexterous Hand is supported. With a hand already running (simulated or HW), run:

```bash
roslaunch sr_leap_motion hand_control.launch
```

This will run the leap nodes, sr_leap_motion nodes, fingertip and wrist teleop nodes, and publish TFs necessary to connect the Leap Motion and Dexterous Hand transform trees.

## Future Work

* Make the Leap hardware work inside a docker container.
* Integrate multiple Leap sensors for increased reliabilty and accuracy.
* Add joint-angle mode (currently only fingertip-position IK is supported)
