/**
 * @file flyMS.cpp
 * @brief flyMS program source code.
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "flyMS/flyMS.hpp"

#include <sys/stat.h>
#include <stdexcept>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

flyMS::flyMS(const YAML::Node &config_params) :
  firstIteration(true),
  ulog_(),
  imuModule(config_params),
  setpointModule(config_params),
  gpsModule(config_params) {
    // Flight mode, 1 = stabilized, 2 = acro
    flight_mode_ = config_params["flight_mode"].as<int>();
    is_debug_mode_ = config_params["debug_mode"].as<bool>();
    log_filepath_ = config_params["log_filepath"].as<std::string>();
    delta_t_ = config_params["delta_t"].as<float>();

    std::array<float, 2> throttle_limits = config_params["setpoint"]["throttle_limits"].as<
      std::array<float, 2> >();
    min_throttle_ = throttle_limits[0];

    YAML::Node controller = config_params["controller"];
    max_control_effort_ = controller["max_control_effort"].as<std::array<float, 3> >();
    roll_PID_inner_coeff_ = controller["roll_PID_inner"].as<std::array<float, 3> >();
    roll_PID_outer_coeff_ = controller["roll_PID_outer"].as<std::array<float, 3> >();
    pitch_PID_inner_coeff_ = controller["pitch_PID_inner"].as<std::array<float, 3> >();
    pitch_PID_outer_coeff_ = controller["pitch_PID_outer"].as<std::array<float, 3> >();
    yaw_PID_coeff_ = controller["yaw_PID"].as<std::array<float, 3> >();

    YAML::Node filters = config_params["filters"];
    imu_lpf_num_ = filters["imu_lpf_num"].as<std::vector<float> >();
    imu_lpf_den_ = filters["imu_lpf_den"].as<std::vector<float> >();
}

//Default Destructor
flyMS::~flyMS() {
  // Free all the memory used in our digital filters
  if (roll_inner_PID_) {
    free(roll_inner_PID_);
  }
  if (roll_outer_PID_) {
    free(roll_outer_PID_);
  }
  if (pitch_inner_PID_) {
    free(pitch_inner_PID_);
  }
  if (pitch_outer_PID_) {
    free(pitch_outer_PID_);
  }
  for (int i = 0; i < 3; i++) {
    if (gyro_lpf_[i]) {
      free(gyro_lpf_[i]);
    }
  }

  //Join the thread if executing
  if (this->flightcoreThread.joinable())
    this->flightcoreThread.join();
}

int flyMS::flightCore() {
  //Local Variables
  int i = 0;
  uint64_t timeStart;
  uint64_t timeFinish;

  rc_set_state(RUNNING);

  while (rc_get_state() != EXITING) {
    /******************************************************************
    *           Grab the time for Periphal Apps and Logs              *
    ******************************************************************/
    timeStart = this->getTimeMicroseconds();
    // printf("time diff start %" PRIu64 "\n", (this->getTimeMicroseconds() - timeStart));

    /******************************************************************
    *         Read, Parse, and Translate IMU data for Flight          *
    ******************************************************************/
    this->imuModule.update();
    this->imuModule.getImuData(&this->imuData);

    // printf("time diff imu %" PRIu64 "\n", (this->getTimeMicroseconds() - timeStart));

    /******************************************************************
    *             Take Care of Some Initialization Tasks              *
    ******************************************************************/

    if (this->firstIteration) {
      this->setpointData.euler_ref[2] = this->imuData.euler[2];
      this->setpointData.yaw_ref_offset = this->imuData.euler[2];
      this->firstIteration = false;
    }

    /************************************************************************
    *                          Get Setpoint Data                            *
    ************************************************************************/
    this->setpointModule.getSetpointData(&this->setpointData);

    /************************************************************************
    *                       Throttle Controller                             *
    ************************************************************************/

    //  float throttle_compensation = 1 / cos(this->imuData.euler[0]);
    //  throttle_compensation *= 1 / cos(this->imuData.euler[1]);

    // if(function_control.altitudeHold)
    // {
    //   if(function_control.altitudeHoldFirstIteration)
    //   {
    //     // this->control.standing_throttle = this->setpointData.throttle;
    //     // this->setpointData.altitudeSetpoint = flyMSData.imu.baro_alt;
    //     // function_control.altitudeHoldFirstIteration = 0;
    //   }
    //       // this->setpointData.altitudeSetpoint=this->setpointData.altitudeSetpoint+(this->setpointData.altitudeSetpointRate)*DT;

    //   //this->control.uthrottle = update_filter(this->filters.altitudeHoldPID, this->setpointData.altitudeSetpoint - flyMSData.baro_alt);
    //   //this->setpointData.throttle = this->control.uthrottle + flyMSData.standing_throttle;
    // }

    /************************************************************************
    *                         Roll Controller                            *
    ************************************************************************/
    if (flight_mode_ == 1) // Stabilized Flight Mode
      this->setpointData.droll_setpoint = update_filter(roll_outer_PID_, this->setpointData.euler_ref[0] - this->imuData.euler[0]);
    else if (flight_mode_ == 2) // Acro mode
      this->setpointData.droll_setpoint = this->setpointData.euler_ref[0];
    else {
      spdlog::error("[flyMS] Error! Invalid flight mode. Shutting down now");
      rc_set_state(EXITING);
      return -1;
    }

    this->imuData.eulerRate[0] = update_filter(gyro_lpf_[0], this->imuData.eulerRate[0]);
    this->control.u_euler[0] = update_filter(roll_inner_PID_, this->setpointData.droll_setpoint - this->imuData.eulerRate[0]);
    this->control.u_euler[0] = saturateFilter(this->control.u_euler[0], -max_control_effort_[0], max_control_effort_[0]);

    /************************************************************************
    *                         Pitch Controller                          *
    ************************************************************************/
    if (flight_mode_ == 1) // Stabilized Flight Mode
      this->setpointData.dpitch_setpoint = update_filter(pitch_outer_PID_, this->setpointData.euler_ref[1] - this->imuData.euler[1]);
    else if (flight_mode_ == 2) // Acro mode
      this->setpointData.dpitch_setpoint = this->setpointData.euler_ref[1];


    this->imuData.eulerRate[1] = update_filter(gyro_lpf_[1], this->imuData.eulerRate[1]);
    this->control.u_euler[1] = update_filter(pitch_inner_PID_, this->setpointData.dpitch_setpoint - this->imuData.eulerRate[1]);
    this->control.u_euler[1] = saturateFilter(this->control.u_euler[1], -max_control_effort_[1], max_control_effort_[1]);

    /************************************************************************
    *                          Yaw Controller                              *
    ************************************************************************/
    this->imuData.eulerRate[2] = update_filter(gyro_lpf_[2], this->imuData.eulerRate[2]);
    this->control.u_euler[2] = update_filter(yaw_PID_, this->setpointData.euler_ref[2] - this->imuData.euler[2]);

    /************************************************************************
    *                     Apply the Integrators                           *
    ************************************************************************/
    if(this->setpointData.throttle < min_throttle_ + .01){
      integrator_reset_++;
    }else{
      integrator_reset_ = 0;
    }

    // if landed for 4 seconds, reset integrators and Yaw error
    if (integrator_reset_ > 4 / delta_t_) {
      this->setpointData.euler_ref[2]=this->imuData.euler[2];
      zeroFilter(roll_inner_PID_);
      zeroFilter(roll_outer_PID_);
      zeroFilter(pitch_inner_PID_);
      zeroFilter(pitch_outer_PID_);
      zeroFilter(yaw_PID_);
    }

    //Apply a saturation filter
    this->control.u_euler[2] = saturateFilter(this->control.u_euler[2], -max_control_effort_[2],
      max_control_effort_[2]);

    /************************************************************************
    *  Mixing
    *
    *                          CCW 1    2 CW    IMU Orientation:
    *                               \ /              Y
    *                               / \              |_ X
    *                           CW 3   4 CCW
    *
    ************************************************************************/

    this->control.u[0] = this->setpointData.throttle - this->control.u_euler[0] + this->control.
      u_euler[1] - this->control.u_euler[2];
    this->control.u[1] = this->setpointData.throttle + this->control.u_euler[0] + this->control.
      u_euler[1] + this->control.u_euler[2];
    this->control.u[2] = this->setpointData.throttle - this->control.u_euler[0] - this->control.
      u_euler[1] + this->control.u_euler[2];
    this->control.u[3] = this->setpointData.throttle + this->control.u_euler[0] - this->control.
      u_euler[1] - this->control.u_euler[2];

    /************************************************************************
    *             Check Output Ranges, if outside, adjust                 *
    ************************************************************************/
    this->check_output_range(this->control.u);

    // printf("time diff control %" PRIu64 "\n", (this->getTimeMicroseconds() - timeStart));
    /************************************************************************
    *                  Send Commands to ESCs                         *
    ************************************************************************/
    if (!is_debug_mode_) {
      std::vector<float> sendVec(4, 0.0);
      //Send Commands to Motors
      for (i = 0; i < 4; i++) {
        sendVec[i] = this->control.u[i];
      }
      pruClientSender.setSendData(sendVec);
    }

    /************************************************************************
    *             Check the kill Switch and Shutdown if set               *
    ************************************************************************/
    if (this->setpointData.kill_switch[0] < 0.5 && !is_debug_mode_) {
      spdlog::info("\nKill Switch Hit! Shutting Down\n");
      rc_set_state(EXITING);
    }

    //Print some stuff to the console in debug mode
    if (is_debug_mode_) {
      this->console_print();
    }
    /************************************************************************
    *             Check for GPS Data and Handle Accordingly               *
    ************************************************************************/
    this->gpsModule.getGpsData(&this->gpsData);

    /************************************************************************
    *               Log Important Flight Data For Analysis              *
    ************************************************************************/
    struct ULogFlightMsg flight_msg(getTimeMicroseconds(), imuData, setpointData, control.u,
      control.u_euler);
    ulog_.WriteFlightData<struct ULogFlightMsg>(flight_msg, FLIGHT_MSG_ID);

    timeFinish = this->getTimeMicroseconds();
    uint64_t sleep_time = static_cast<uint64_t>(delta_t_*1.0E6) - (timeFinish - timeStart);

    // Check to make sure the elapsed time wasn't greater than time allowed.
    // If so don't sleep at all
    if (sleep_time < static_cast<uint64_t>(delta_t_*1.0E6))  rc_usleep(sleep_time);
    else spdlog::warn("[flyMS] Error! Control thread too slow! time in micro seconds: {}",
      (timeFinish - timeStart));

  }
  return 0;
}


uint64_t flyMS::getTimeMicroseconds() {
  struct timespec tv;
  clock_gettime(CLOCK_MONOTONIC, &tv);
  return (uint64_t)tv.tv_sec * 1E6 + (uint64_t)tv.tv_nsec / 1E3;
}


int flyMS::console_print() {
//  spdlog::info("time {:3.3f} ", control->time);
//  spdlog::info("Alt_ref {:3.1f} ",control->alt_ref);
//  spdlog::info(" U1:  {:2.2f} ",control->u[0]);
//  spdlog::info(" U2: {:2.2f} ",control->u[1]);
//  spdlog::info(" U3:  {:2.2f} ",control->u[2]);
//  spdlog::info(" U4: {:2.2f} ",control->u[3]);
//  spdlog::info("Aux %2.1f ", control->setpoint.Aux[0]);
//  spdlog::info("function: {}",rc_get_dsm_ch_normalized(6));
//  spdlog::info("num wraps {} ",control->num_wraps);
  // spdlog::info(" Throt {:2.2f} ", this->setpointData.throttle);
  // spdlog::info(" Roll_ref {:2.2f} ", this->setpointData.euler_ref[0]);
  // spdlog::info(" Pitch_ref {:2.2f} ", this->setpointData.euler_ref[1]);
  // spdlog::info(" Yaw_ref {:2.2f} ", this->setpointData.euler_ref[2]);
  spdlog::info("Roll {:1.2f}, Pitch {:1.2f}, Yaw {:2.3f}", this->imuData.euler[0],
    this->imuData.euler[1], this->imuData.euler[2]);
//  spdlog::info(" Mag X {:4.2f}",control->mag[0]);
//  spdlog::info(" Mag Y {:4.2f}",control->mag[1]);
//  spdlog::info(" Mag Z {:4.2f}",control->mag[2]);
  // spdlog::info(" Accel X {:4.2f}",control->accel[0]);
  // spdlog::info(" Accel Y {:4.2f}",control->accel[1]);
  // spdlog::info(" Accel Z {:4.2f}",control->accel[2]);
//   spdlog::info(" Pos N {:2.3f} ", control->ekf_filter.output.ned_pos[0]);
//  spdlog::info(" Pos E {:2.3f} ", control->ekf_filter.output.ned_pos[1]);
//  spdlog::info(" Pos D {:2.3f} ", control->ekf_filter.output.ned_pos[2]);
  // spdlog::info(" DPitch {:1.2f} ", control->euler_rate[0]);
  // spdlog::info(" DRoll {:1.2f} ", control->euler_rate[1]);
  // spdlog::info(" DYaw {:2.3f} ", control->euler_rate[2]);
//  spdlog::info(" uyaw {:2.3f} ", control->upitch);
//  spdlog::info(" uyaw {:2.3f} ", control->uroll);
//  spdlog::info(" uyaw {:2.3f} ", control->uyaw);
//  spdlog::info(" GPS pos lat: {:2.2f}", control->GPS_data.pos_lat);
//  spdlog::info(" GPS pos lon: {:2.2f}", control->GPS_data.pos_lon);
//  spdlog::info(" HDOP: {}", control->GPS_data.HDOP);
//  spdlog::info("Baro Alt: {} ",control->baro_alt);
  return 0;
}

int flyMS::check_output_range(float u[4]) {
  int i;
  float largest_value = 1;
  float smallest_value = 0;

  for (i = 0; i < 4; i++) {
    if (u[i] > largest_value) largest_value = u[i];

    if (u[i] < smallest_value) u[i] = 0;
  }

  // if upper saturation would have occurred, reduce all outputs evenly
  if (largest_value > 1) {
    float offset = largest_value - 1;
    for (i = 0; i < 4; i++) u[i] -= offset;
  }
  return 0;
}

std::string flyMS::GetLogDir(const std::string &log_location) {
  int run_number = 1;
  std::string run_folder(log_location + std::string("/run") + std::to_string(run_number));

  //Find the next run number folder that isn't in use
  struct stat st = {0};
  while (!stat(run_folder.c_str(), &st)) {
    run_folder = (log_location + std::string("/run") + std::to_string(++run_number));
  }

  //Make a new folder to hold the logged data
  mkdir(run_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  return run_folder;
}

int flyMS::InitializeSpdlog(const std::string &log_dir) {
  int max_bytes = 1048576 * 50;  // Max 20 MB
  int max_files = 20;

  std::vector<spdlog::sink_ptr> sinks;
  // Only use the console sink if we are in debug mode
  if (is_debug_mode_) {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  }
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt> (
    log_dir + "/console_log.txt", max_bytes, max_files));
  auto flyMS_log = std::make_shared<spdlog::logger>("flyMS_log", std::begin(sinks),
    std::end(sinks));

  //register it if you need to access it globally
  flyMS_log->set_level(spdlog::level::trace);
  spdlog::register_logger(flyMS_log);
  spdlog::set_default_logger(flyMS_log);
}

int flyMS::startupRoutine() {
  //Initialize the remote controller through the setpoint object
  if (this->setpointModule.start())
    std::cerr << "[flyMS] Error initializing Radio Coms!" << std::endl;

  std::cout << "[flyMS] Starting ready check!!" << std::endl;
  //Pause the program until the user toggles the kill switch
  if (!is_debug_mode_) {
    if (this->readyCheck()) {
      printf("Exiting Program \n");
      return -1;
    }
  }
  std::cout << "[flyMS] Done ready check!!" << std::endl;

  //Tell the system that we are running
  rc_set_state(RUNNING);

  // Create a file for logging and initialize our file logger
  std::string log_dir = GetLogDir(log_filepath_);
  InitializeSpdlog(log_dir);
  ulog_.InitUlog(log_dir);

  //Initialize the PID controllers and LP filters
  this->initializeFilters();

  //Tell the setpoint manager we are no longer waiting to initialize
  this->setpointModule.setInitializationFlag(false);

  //Initialize the IMU Hardware
  if (this->imuModule.initializeImu())
    return -1;

  //Initialize the client to connect to the PRU handler
  this->pruClientSender.startPruClient();

  //Start the flight program
  this->flightcoreThread = std::thread(&flyMS::flightCore, this);

  return 0;
}


int flyMS::readyCheck() {
  //Toggle the kill switch to get going, to ensure controlled take-off
  //Keep kill switch down to remain operational
  int count = 1, toggle = 0, reset_toggle = 0;
  float val[2] = {0.0f , 0.0f};
  bool firstRun = true;
  printf("Toggle the kill swtich twice and leave up to initialize\n");
  while (count < 6 && rc_get_state() != EXITING) {
    // Blink the green LED light to signal that the program is ready
    reset_toggle++; // Only blink the led 1 in 20 times this loop runs
    if (toggle) {
      rc_led_set(RC_LED_GREEN, 0);
      if (reset_toggle == 20) {
        toggle = 0;
        reset_toggle = 0;
      }
    } else {
      rc_led_set(RC_LED_GREEN, 1);
      if (reset_toggle == 20) {
        toggle = 1;
        reset_toggle = 0;
      }
    }

    if (this->setpointModule.getSetpointData(&this->setpointData)) {
      //Skip the first run to let data history fill up
      if (firstRun) {
        firstRun = false;
        continue;
      }
      val[1] = this->setpointData.kill_switch[1];
      val[0] = this->setpointData.kill_switch[0];

      if (val[0] < 0.25 && val[1] > 0.25)
        count++;
      if (val[0] > 0.25 && val[1] < 0.25)
        count++;
    }
    usleep(10000);
  }

  //make sure the kill switch is in the position to fly before starting
  while (val[0] < 0.25 && rc_get_state() != EXITING) {
    if (this->setpointModule.getSetpointData(&this->setpointData)) {
      val[1] = this->setpointData.kill_switch[1];
      val[0] = this->setpointData.kill_switch[0];
    }
    usleep(10000);
  }

  if (rc_get_state() == EXITING) {
    printf("State set to exiting, shutting off! \n");
    return -1;
  }

  printf("\nInitialized! Starting program\n");
  rc_led_set(RC_LED_GREEN, 1);
  return 0;
}


/************************************************************************
*  initialize_filters()
*  setup of feedback controllers used in flight core
************************************************************************/
int flyMS::initializeFilters() {
  roll_outer_PID_  = generatePID(roll_PID_outer_coeff_[0], roll_PID_outer_coeff_[1],
    roll_PID_outer_coeff_[2], 0.15, delta_t_);
  pitch_outer_PID_ = generatePID(pitch_PID_outer_coeff_[0], pitch_PID_outer_coeff_[1],
    pitch_PID_outer_coeff_[2], 0.15, delta_t_);
  yaw_PID_ = generatePID(yaw_PID_coeff_[0], yaw_PID_coeff_[1], yaw_PID_coeff_[2], 0.15, delta_t_);

  roll_inner_PID_  = generatePID(roll_PID_inner_coeff_[0], roll_PID_inner_coeff_[0],
    roll_PID_inner_coeff_[0], 0.15, delta_t_);
  pitch_inner_PID_ = generatePID(pitch_PID_inner_coeff_[0], pitch_PID_inner_coeff_[1],
    pitch_PID_inner_coeff_[2], 0.15, delta_t_);

  // Check to make sure our digital filter is a correct size
  if (imu_lpf_num_.size() != imu_lpf_den_.size()) {
    spdlog::error("Low pass filter coefficients do not match in size!");
    throw std::invalid_argument("Low pass filter coefficients do not match in size!");
  }

  // Initialize the filters
  for (int i = 0; i < 3; i++) {
    // accel_lpf_[i] = initialize_filter(imu_lpf_num_.size(), imu_lpf_num_.data(),
      // imu_lpf_den_.data());
    gyro_lpf_[i] = initialize_filter(imu_lpf_num_.size(), imu_lpf_num_.data(),
      imu_lpf_den_.data());
  }
  return 0;
}
