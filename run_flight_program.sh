#!/bin/bash

j="1"


while getopts r option
do
 case "${option}"
 in
 r) j="30";;
 esac
done




i="0"
while [ $i -lt $j ]
do
	echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/unbind
	echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/bind

	echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/unbind
	echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/bind

	flyMS > /dev/null
	((i++))
done


