#!/bin/bash


pru_handler &

while [ ! -f /var/run/pru_handler.pid ]
do
  sleep 1
done

sleep 1

j="20"


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
	nohup 	flyMS
	((i++))
	sleep 1
done


