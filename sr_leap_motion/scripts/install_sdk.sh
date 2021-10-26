#!/bin/bash

if hash leapd 2>/dev/null; then
    echo "Leap SDK is already installed."
else
    sudo apt update && \
    sudo apt install -y wget && \
    wget https://s3.eu-west-2.amazonaws.com/com.shadowrobot.eu-west-2.public/leap_motion/Leap-2.3.1%2B31549-x64.deb && \
    sudo apt install -y ./Leap*x64.deb && \
    rm ./Leap*x64.deb
fi
