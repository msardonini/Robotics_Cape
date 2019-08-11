#!/bin/bash

# Parse command line arguments

# Call getopt to validate the provided input. 
options=$(getopt -o c --long address: -- "$@")
[ $? -eq 0 ] || { 
  echo "Incorrect options provided"
  exit 1
}
eval set -- "$options"
while true; do
  case "$1" in
  -c)
    # clean the build directory before compiling
    CLEAN=1
    ;;
  --address)
    shift; # The arg is next in position args
    ADDRESS=$1
    ;;
  --)
    shift
    break
    ;;
  esac
  shift
done


# Delete the build folder if it exists
[ $CLEAN -eq 1 ] && [ -d "build" ] && rm -rf build
[ -d "build" ] && mkdir build
cd build
cmake ..
make -j$('nproc')
cd ../

# Determine if we are running on the embedded device or a host x86_64 machine
ARCH="$(uname -m)"

if [ $ARCH == "x86_64" ]; then
  # insctructions for copying files over to the embedded device
  if [ -z $ADDRESS ]; then
    RED='\033[0;31m'
    NC='\033[0m' # No Color
    printf "${RED}Error!${NC} Invalid beaglebone address given!\n"
    exit 1
  fi
  scp build/bin/* debian@$ADDRESS:/home/debian/bin
elif [ $ARCH == "arm" ]; then 
  cp build/bin /home/debian/bin
else
    echo "Invalid CPU architecture!"
    exit 1
fi

