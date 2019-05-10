/**
 * @file flyMS.cpp
 * @brief Application entry point of the flyMS program.
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

//System Includes
#include <signal.h>
#include <getopt.h>

//Our Includes
#include "src/flyMS.hpp"

void initSignalHandler();
void onSignalReceived(int signo);


int main(int argc, char *argv[]) {
  //Enable the signal handler so we can exit cleanly on SIGINT
  initSignalHandler();

  // Read the command line arguments and config file inputs
  flyMSParams configParams;

  bool isDebugMode = false;
  std::string configFilepath;
  //Parse the command line arguments
  int in;
  while ((in = getopt(argc, argv, "c:dr:h")) != -1) {
    switch (in) {
    case 'c':
      configFilepath = std::string(optarg);
      break;
    case 'd':
      isDebugMode = true;
      printf("Running in Debug mode \n");
      break;
    case 'r':
      //Run Program in replay mode, need to provide filepath to log file
      //TODO: Add command line arg parser
      printf("Running in Replay mode \n");
      break;
    case 'h':
      //Display the command line help options
      //TODO: make this function
      // print_usage();
      return 0;
    default:
      printf("Invalid Argument \n");
      return -1;
    }
  }

  // Make sure the user provided a path to a confile file
  if (configFilepath.empty()) {
    std::cout << "Reqired parameter: -c" << std::endl;
    return -1;
  }

  configParams.loadConfigFile(configFilepath);

  //Merge the common parameters between the config file and the command line inputs
  configParams.config.isDebugMode |= isDebugMode;

  rc_set_state(UNINITIALIZED);
  // flyMS fly;
  flyMS fly(configParams);
  //Initialize the flight hardware
  if (fly.startupRoutine())
    rc_set_state(EXITING);

  //Reload the config file in case changes were made while waiting

  while (rc_get_state() != EXITING) {
    sleep(1);
  }

  // rc_cleanup();
  return 0;
}

void onSignalReceived(int signo) {
  switch (signo) {
  case SIGHUP:
    break;
  default:
    rc_set_state(EXITING);
  }
}

void initSignalHandler() {
  signal(SIGINT, onSignalReceived);
  signal(SIGKILL, onSignalReceived);
  signal(SIGHUP, onSignalReceived);
}
