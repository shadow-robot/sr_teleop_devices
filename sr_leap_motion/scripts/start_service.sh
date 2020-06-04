#!/bin/bash

# If we're inside a docker container, we may need to start dbus and manually manage leapd's lifecycle
if [ -f /.dockerenv ]; then
    sudo /etc/init.d/dbus start
    sudo pkill -9 leapd
    sudo leapd &
fi
# If not, we assume the user has set up host running as per the README.
