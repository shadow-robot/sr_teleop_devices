#!/bin/bash
# Copyright 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# If we're inside a docker container, we may need to start dbus and manually manage leapd's lifecycle
if [ -f /.dockerenv ]; then
    if sudo /etc/init.d/dbus status > /dev/null; then
        echo "D-Bus is running"
    else
        echo "Starting D-bus..."
        if sudo /etc/init.d/dbus start > /dev/null; then
            echo "D-bus started."
        else
            echo "Failed to start D-bus!"
            exit 1
        fi
    fi
    if pgrep -x "leapd" > /dev/null; then
        echo "Leapd is already running."
    else
        echo "Starting Leapd..."
        sudo -b leapd > /dev/null
        if (( $? == 0 )); then
            echo "Leapd started."
        else
            echo "Failed to start Leapd"
            exit 1
        fi
    fi
fi
exit 0

# If not, we assume the user has set up host running as per the README.
