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

if hash leapd 2>/dev/null; then
    echo "Leap SDK is already installed."
else
    sudo apt update && \
    sudo apt install wget && \
    wget https://s3.eu-west-2.amazonaws.com/com.shadowrobot.eu-west-2.public/leap_motion/Leap-2.3.1%2B31549-x64.deb && \
    sudo apt install ./Leap*x64.deb && \
    rm ./Leap*x64.deb
fi
