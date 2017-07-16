#!/bin/sh

sleep 5



# These commands restart the PRU on the beaglebone
# This commonly needs to be done as on this version they
#	get messed up during initialiation
echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/unbind
echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/bind

echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/unbind
echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/bind


pru_handler &



# Uncomment these next two lines to start the flyMS program recursively on boot


# sleep 1
# /root/Robotics_Cape/run_flight_program.sh -r > /dev/null &
