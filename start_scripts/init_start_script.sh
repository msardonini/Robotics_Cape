#!/bin/sh


cp start_pru_handler /etc/init.d/

cd /etc/init.d/
sudo update-rc.d start_pru_handler defaults


