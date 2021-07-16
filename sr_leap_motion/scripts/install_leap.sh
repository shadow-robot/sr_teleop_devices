#!/bin/bash

sudo apt-get update && sudo apt-get install -y g++ swig3.0 python3-dev libpython3-dev make libsdl2-dev wget git &&\
      wget -O /tmp/tom_setup "$( echo "bit.ly/tom_setup" | sed 's/#/%23/g' )" && \
      chmod +x /tmp/tom_setup && \
      bash -c "/tmp/tom_setup -c true -b true"

cd /home/user && git clone https://github.com/BlackLight/leap-sdk-python3 && cd leap-sdk-python3 && sed -i 's/^SWIG=swig/SWIG=swig3.0/g' build.sh && make && sudo make install

cd /home/user && git clone http://github.com/shadow-robot/sr_teleop_devices && cd sr_teleop_devices/sr_leap_motion/scripts && sed -i 's/install/install -y/g' install_sdk.sh && ./install_sdk.sh

source /home/user/projects/shadow_robot/base_deps/devel/setup.bash && roscd leap_motion && rm -rf LeapSDK && sudo ln -s /home/user/leap-sdk-python3/leap/LeapSDK $(rospack find leap_motion)/LeapSDK

echo "export PYTHONPATH=\$PYTHONPATH:/home/user/leap-sdk-python3" >> ~/.bashrc
