#!/bin/sh

sleep 5

echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/unbind
echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/bind

echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/unbind
echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/bind


pru_handler &


sleep 1

/root/Robotics_Cape/run_flight_program.sh -r
