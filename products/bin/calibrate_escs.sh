#!/bin/bash

sudo systemctl stop flyMS.service
sudo systemctl stop pruHandler.service

sudo rc_calibrate_escs

sudo systemctl start pruHandler.service
sudo systemctl start flyMS.service
