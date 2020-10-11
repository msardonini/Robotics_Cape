# Beaglebone QuadCopter with Robotics Cape API
	Quadcopter flight program for Beaglebone Blue or Beaglebone Black with Robotics Cape. 
	Built with StrawsonDesign's Robotics Cape Library http://strawsondesign.com/#!manual-install.
	

## Allow access to enable real time threads
In order to run the flyMS program you need to allow the "debian" user (or whatever user you are using) to spawn real time threads. You can do this by adding these lines to the file "/etc/security/limits.conf"

```
debian hard rtprio 99
debian soft rtprio 99
```

A reboot is required after making this change


## Prereq's
** Note that these prereq's are installed automatically with the install.sh script

	1. cmake
	2. Eigen

## First Install

1. Log in as root (needs to be enabled, https://linuxconfig.org/enable-ssh-root-login-on-debian-linux-server)
  ```
$  cd /root/ 
$  git clone --recursive git@github.com:msardonini/Robotics_Cape.git
$  cd Robotics_Cape/
$  bash install.sh
```
2. Reboot Beaglebone

### Subsequent Builds (Updates to source, etc)
```
$	cd /root/Robotics_Cape/build/
$	cmake ..
$	make install

```

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

### Motor Spin Directions & ESC connections  

  CCW 1	&nbsp;  2 CW			<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; \ &nbsp; /	<br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; / &nbsp; \  	<br />
 CW 3 &nbsp; &nbsp;	   4 CCW		<br />
	





