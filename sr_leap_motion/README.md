# sr_leap_motion

This package contains an interface between Shadow software and Leap motion software. It relies on the leap_motion package for `.msg` definitions.

## Prerequisites

Whether running inside or outside of a container, there are also the usual ROS prerequisite steps (rosdep to install binary dependencies listed in [package.xml](package.xml), wstool and catkin to fetch and install src dependencies). If you are running in a docker container that had this repository pre-built, this should have been handled for you already.

## Running outside of docker

Run [install_leap.sh](scripts/install_leap.sh); this will install the Leap Motion SDK V2 for Linux and a [python3 sdk](https://github.com/BlackLight/leap-sdk-python3). Afterwards, you will need to restart your terminal or start a new one in order for Python to find the `Leap` module.

The Leap service (leapd) must be running all the time; Ubuntu has changed how services work since V2 was released. As a result, you need to run [install_systemctl.sh](scripts/install_systemctl.sh)

## Running inside docker

Surprisingly, no udev rules or `/dev` forwarding shenanigans are required. Just make sure the Leap Motion is plugged into your machine before starting the container. You can stop and start the container to achieve this, it doesn't need to be a fresh `docker run`.

Run [install_leap.sh](scripts/install_leap.sh); this will install the Leap Motion SDK V2 for Linux and a [python3 sdk](https://github.com/BlackLight/leap-sdk-python3) inside your container. Afterwards, you will need to restart your terminal or start a new one in order for Python to find the `Leap` module.

You don't need to set up the `leapd` service when running inside a container; a script started in [leap_motion.launch](launch/leap_motion.launch) which takes care of the service lifecycle for you.

## Running

### Stand-alone

`sr_leap_motion.launch` is the main launchfile - it will start Leap nodes and publish user hand TFs and a heartbeat topic. You can run it stand-alone and visualise the data by running:

```bash
roslaunch sr_leap_motion sr_leap_motion.launch rviz:=true
```

### Controlling a Shadow Dexterous Hand

Currently only a right Dexterous Hand is supported. With a hand already running (simulated or HW), run:

```bash
roslaunch sr_fingertip_hand_teleop leap_motion.launch
```

This requires access to the private sr_teleop_internal repository. It will run the leap service, sr_leap_motion nodes, fingertip and wrist teleop nodes, and publish TFs necessary to connect the Leap Motion and Dexterous Hand transform trees.

## Future Work

* Make the Leap hardware work inside a docker container.
* Integrate multiple Leap sensors for increased reliabilty and accuracy.
* Add joint-angle mode (currently only fingertip-position IK is supported)
* Make left hand work
