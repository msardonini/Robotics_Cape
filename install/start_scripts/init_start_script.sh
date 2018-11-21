#!/bin/sh


sudo cp flyMS.service /lib/systemd/system/
sudo cp pruHandler.service /lib/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable flyMS
sudo systemctl enable pruHandler


