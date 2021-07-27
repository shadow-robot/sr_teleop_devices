#!/bin/bash

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
# source /home/user/projects/shadow_robot/base_deps/devel/setup.bash && roscd leap_motion && rm -rf LeapSDK && sudo ln -s /opt/leap/leap-sdk-python3/leap/LeapSDK $(rospack find leap_motion)/LeapSDK
temp_file=$(mktemp) && for x in $(printf '%s' "$ROS_PACKAGE_PATH" | sed -r 's/:/\n/g'); do echo $x; done > $temp_file && for x in $(awk '{a[i++]=$0} END {for (j=i-1; j>=0;) print a[j--] }' $temp_file); do if [[ $(basename $x) == "src" ]]; then echo $x; source $x/../devel/setup.bash; else echo $x; source $x/../setup.bash; fi; done && roscd leap_motion && rm -rf LeapSDK && sudo ln -s /opt/leap/leap-sdk-python3/leap/LeapSDK $(rospack find leap_motion)/LeapSDK
echo "export PYTHONPATH=\$PYTHONPATH:/opt/leap/leap-sdk-python3" >> ~/.bashrc
