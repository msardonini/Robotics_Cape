#!/bin/sh

sudo cp services/flyMS.service /etc/systemd/system/
sudo cp services/pruHandler.service /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable flyMS
sudo systemctl enable pruHandler
