#!/bin/sh

sudo rm /etc/systemd/system/flyMS.service
sudo rm /etc/systemd/system/pruHandler.service

sudo systemctl disable flyMS
sudo systemctl disable pruHandler
sudo systemctl daemon-reload
