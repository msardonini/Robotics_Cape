#!/bin/sh


cp restart_pru.sh /etc/init.d/

cd /etc/init.d/
sudo update-rc.d restart_pru.sh defaults


