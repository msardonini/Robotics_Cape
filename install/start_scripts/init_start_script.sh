#!/bin/sh


sudo cp flyMS.service /etc/systemd/system/
sudo cp pruHandler.service /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable flyMS
sudo systemctl enable pruHandler


