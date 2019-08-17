#!/bin/bash

# TODO: add option to copy config file too
# TODO: clean up other scripts in this directory

function print_usage {
  echo "flyMS_build.sh"
  echo "  Builds the flyMS flight program and dependencies on either a "
  echo "  beaglebone or host x86_64 machine (requires cross compiler)"
  echo "Usage:"
  echo "  ./flyMS_build.sh [OPTIONS] "
  echo "    OPTIONS:"
  echo "      -c                      Cleans the build directory before making again"
  echo "      --address {address}     Specify the IP address of the beaglebone to"
  echo "                              send outputs to. Required for cross compiling"
  echo "      --config {config_file}  Path to a config file to use for flight. This "
  echo "                              will overwrite an existing config file"
  echo "      -h                      Prints this help menu and exits"
}

function prep_output_folder {
  mkdir products
  cp -r build/bin products
  cp -r scripts/* products/bin
  [ ! -z $1 ] && cp $1 products/flyMSConfig.yaml
}

function rm_output_folder {
  rm -rf products
}

# Defaults
CLEAN=0
CONFIG_FILE=""
ADDRESS=""

# Parse command line arguments
# Call getopt to validate the provided input. 
options=$(getopt -o ch --long address: --long config: -- "$@")
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
  -h)
    # prints the help menu and exit
    print_usage
    exit 0
    ;;
  --config)
    shift;
    CONFIG_FILE=$1
    echo "config $CONFIG_FILE"
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

# Delete the build folder if it exists and the user requested to clean
[ "$CLEAN" -eq 1 ] && [ -d "build" ] && rm -rf build
# Make a build folder if not present
[ ! -d "build" ] && mkdir build
cd build
cmake ..
make -j$('nproc')
cd ../

# Make a folder to store things to send
prep_output_folder $CONFIG_FILE

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
  DESTINATION="debian@$ADDRESS:/home/debian"
elif [ $ARCH == "armv7l" ]; then 
  DESTINATION="/home/debian"
else
    echo "Invalid CPU architecture!"
    exit 1
fi

# rsync the files over
rsync -auv --info=progress2 products/ debian@$ADDRESS:/home/debian/

rm_output_folder