This directory contains scripts useful for setting up and running a leap motion controller. All can be run with

```bash
rosrun sr_leap_motion <script.sh>
```
`install_leap.sh` builds and installs the python3 leap api, `install_sdk.sh` is called as part of this script.

`install_sdk.sh` installs the Leap Motion SDK by downloading a .deb and install it and it's dependencies.

`install_systemctl.sh` installs a systemctl service to manage the Leap daemon lifecycle.

`start_service.sh` manually starts dbus and the Leap daemon when run inside a container, and is used by launch files to facilitate use in a docker container.

`stop_service.sh` nukes the Leap daemon, which is otherwise quite bad at stopping.
