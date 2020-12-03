#!/bin/bash

sudo cp $(rospack find sr_leap_motion)/resources/leapd.service /lib/systemd/system/.
sudo ln -s /lib/systemd/system/leapd.service /etc/systemd/system/leapd.service
sudo systemctl daemon-reload
sudo systemctl enable leapd
sudo systemctl start leapd
