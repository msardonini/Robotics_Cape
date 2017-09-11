# Robotics Cape for Beaglebone QuadCopter

## Prereq's

	1. cmake
	2. Eigen

## Install

1. Log in as root
  * cd /root/ 
  * git clone --recursive git@github.com:msardonini/Robotics_Cape.git
  * cd Robotics_Cape/
  * mkdir build
  * cmake ..
  * make
  * sudo make install

## Run Program
	1. SSH into beaglebone
	2. Calibrate esc's (if needed)
	3. Run "nohup flyMS"
	4. Disconnect usb wire if attached
	5. Toggle kill switch twice and leave up to initialize
	6. Fly


## Major Changes from Existing Repo
	1. flyMS flight program
	2. "pru_handler" is a background process which runs on boot. This sends zero commands to the escs while no process is running so esc's stop beeping when powered. It also communicates with programs which send esc commands 
	3. Digital Filter improved, minimum memory dynamically allocated and shifting of timestamps is done in place
	4. The estimation and control library (from PX4) has been stripped down and put into this build for GPS/INS fused navigation


### Original Repo 
For BeagleBone Black and BeagleBone Blue running Debian Jessie.

Installation instructions and user manual can be found here:
http://strawsondesign.com/#!manual-install






