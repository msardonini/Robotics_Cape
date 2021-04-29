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
#include <string>
#include <memory>

//Our Includes
#include "yaml-cpp/yaml.h"
#include "flyMS/flyMS.h"
#include "flyMS/ready_check.h"
#include "spdlog/spdlog.h"

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

int main(int argc, char *argv[]) {
  //Enable the signal handler so we can exit cleanly on SIGINT
  initSignalHandler();

  bool is_debug_mode = false;
  std::string config_filepath;
  //Parse the command line arguments
  int in;
  while ((in = getopt(argc, argv, "c:dr:h")) != -1) {
    switch (in) {
    case 'c':
      config_filepath = std::string(optarg);
      break;
    case 'd':
      is_debug_mode = true;
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
  if (config_filepath.empty()) {
    std::cout << "Reqired parameter: -c" << std::endl;
    return -1;
  }

  // Perform the ready check before starting the flight program
  if (!is_debug_mode) {
    ReadyCheck ready_check;
    ready_check.WaitForStartSignal();
  }

  // Load the Yaml Node
  YAML::Node config_params = YAML::LoadFile(config_filepath)["flyMSParams"];

  // Override the debug flag if requested at the command line
  if (is_debug_mode) {
    config_params["debug_mode"] = "true";
  }

  rc_set_state(UNINITIALIZED);

  flyMS fly(config_params);
  //Initialize the flight hardware
  if (fly.StartupRoutine()) {
    rc_set_state(EXITING);
  }

  //Reload the config file in case changes were made while waiting

  while (rc_get_state() != EXITING) {
    sleep(1);
  }

  spdlog::shutdown();
  return 0;
}

