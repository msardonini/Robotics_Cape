#!/bin/sh


#Issue a hard kill to run_fight_program.sh if it's running
ps -ef | grep run_flight | grep -v grep | awk '{print $2}' | xargs kill -9

#Send a SIGINT to flyMS so it can shutdown gracefully, if its running
ps -ef | grep flyMS | grep -v grep | awk '{print $2}' | xargs kill -INT

