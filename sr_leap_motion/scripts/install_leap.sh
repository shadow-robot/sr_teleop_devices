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

USERNAME=$(whoami)
sudo apt-get update && sudo apt-get install -y g++ swig3.0 python3-dev libpython3-dev make libsdl2-dev wget git
cd /opt
sudo mkdir leap
sudo chown -R $USERNAME:$USERNAME leap
sudo chmod 0755 leap
cd leap
git clone https://github.com/BlackLight/leap-sdk-python3 && cd leap-sdk-python3 && sed -i 's/^SWIG=swig/SWIG=swig3.0/g' build.sh && make && sudo make install
cd /opt/leap
git clone http://github.com/shadow-robot/sr_teleop_devices && cd sr_teleop_devices/sr_leap_motion/scripts && sed -i 's/install/install -y/g' install_sdk.sh && ./install_sdk.sh
cd /opt/leap && rm -rf sr_teleop_devices
# The line below sources the top level ros `setup.bash`, allowing roscd and rospack to work
IFS=":" read -a top_path <<< $ROS_PACKAGE_PATH && if [[ $(basename ${top_path[0]}) == "src" ]]; then source ${top_path[0]}/../devel/setup.bash; else source ${top_path[0]}/../setup.bash; fi && roscd leap_motion && rm -rf LeapSDK && sudo ln -s /opt/leap/leap-sdk-python3/leap/LeapSDK $(rospack find leap_motion)/LeapSDK
echo "export PYTHONPATH=\$PYTHONPATH:/opt/leap/leap-sdk-python3" >> ~/.bashrc
