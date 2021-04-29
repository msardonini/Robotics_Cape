/**
 * @file flyMS.cpp
 * @brief flyMS program source code.
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "flyMS/flyMS.h"

#include <sys/stat.h>
#include <pthread.h>
#include <stdexcept>
#include <iomanip>
#include <sstream>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/fmt/ostr.h"

flyMS::flyMS(const YAML::Node &config_params) :
  ulog_(),
  setpoint_module_(),
  gps_module_(config_params),
  mavlink_interface_(config_params),
  config_params_(config_params) {
    // Flight mode, 1 = stabilized, 2 = acro
    flight_mode_ = config_params["flight_mode"].as<int>();
    is_debug_mode_ = config_params["debug_mode"].as<bool>();
    log_filepath_ = config_params["log_filepath"].as<std::string>();
    delta_t_ = config_params["delta_t"].as<float>();

    YAML::Node controller = config_params["controller"];
    max_control_effort_ = controller["max_control_effort"].as<std::array<float, 3> >();

}

// Default Destructor
flyMS::~flyMS() {
  //Join the thread if executing
  if (flightcore_thread_.joinable())
    flightcore_thread_.join();
}

int flyMS::FlightCore() {
  while (rc_get_state() != EXITING) {
    /******************************************************************
    *           Grab the time for Periphal Apps and Logs              *
    ******************************************************************/
    uint64_t timeStart = GetTimeMicroseconds();

    /******************************************************************
    *         Read, Parse, and Translate IMU data for Flight          *
    ******************************************************************/
    imu_module_->GetImuData(&imu_data_);
    mavlink_interface_.SendImuMessage(imu_data_);

    /************************************************************************
    *       Check the Mavlink Interface for New Visual Odometry Data
    ************************************************************************/
    VioData vio;
    Eigen::Vector3f setpoint_orientation;
    float vio_yaw;
    if(mavlink_interface_.GetVioData(&vio)) {
      setpoint_module_->position_controller->ReceiveVio(vio);
      setpoint_module_->position_controller->GetSetpoint(setpoint_orientation, vio_yaw);

      float log_setpoint[4] = {setpoint_orientation(0), setpoint_orientation(1), vio_yaw,
        setpoint_orientation(2)};
       float quat_setpoint[4] = {vio.quat.w(), vio.quat.x(), vio.quat.y(), vio.quat.z()};

      // Log the VIO Data
      ULogPosCntrlMsg vio_log_msg(timeStart, vio.position.data(), vio.velocity.data(),
        quat_setpoint, log_setpoint);
      ulog_.WriteFlightData<ULogPosCntrlMsg>(vio_log_msg, ULogPosCntrlMsg::ID());
      flyStereo_streaming_data_ = true;
    }

    /************************************************************************
    *                          Get Setpoint Data                            *
    ************************************************************************/
    setpoint_module_->GetSetpointData(&setpoint_);

    // If we have commanded a switch in Aux, activate the perception system
    if (setpoint_.Aux[0] < 0.1 && setpoint_.Aux[1] > 0.9) {
      // Transition to start flyStereo
      mavlink_interface_.SendStartCommand();

      // Assume that the current throttle value will be an average value to keep altitude
      standing_throttle_ = setpoint_.throttle;
      initial_yaw_ = imu_data_.euler[2];

      // Make sure our first setpoint is zero
      flyStereo_running_ = true;
    } else if (setpoint_.Aux[0] > 0.9 && setpoint_.Aux[1] < 0.1) {
      flyStereo_running_ = false;
      flyStereo_streaming_data_ = false;
      mavlink_interface_.SendShutdownCommand();

      // Reset the filters in the Position controller
      setpoint_module_->position_controller->ResetController();
      setpoint_orientation = Eigen::Vector3f::Zero();
      position_generator_.ResetCounter();
    }

    // Apply orientation commands from the position controller if running
    if (flyStereo_running_ && flyStereo_streaming_data_) {
      setpoint_.euler_ref[0] = setpoint_orientation(0);
      setpoint_.euler_ref[1] = setpoint_orientation(1);

      setpoint_.throttle = setpoint_orientation(2) + standing_throttle_;
      // setpoint_.euler[2] = vio_yaw - imu_data_.euler[2] + initial_yaw_;

      // Apply the position setpoint
      Eigen::Vector3f pos_ref;
      position_generator_.GetPosition(&pos_ref);
      setpoint_module_->position_controller->SetReferencePosition(pos_ref);
    }

    /************************************************************************
    *                         Roll Controller                               *
    ************************************************************************/
    float droll_setpoint;
    if (flight_mode_ == 1) {  // Stabilized Flight Mode
      droll_setpoint = update_filter(flight_filters_->roll_outer_PID,
        setpoint_.euler_ref[0] - imu_data_.euler[0]);
    } else if (flight_mode_ == 2) {  // Acro mode
      droll_setpoint = setpoint_.euler_ref[0];
    } else {
      spdlog::error("[flyMS] Error! Invalid flight mode. Shutting down now");
      rc_set_state(EXITING);
      return -1;
    }

    imu_data_.eulerRate[0] = update_filter(flight_filters_->gyro_lpf[0], imu_data_.eulerRate[0]);
    u_euler_[0] = update_filter(flight_filters_->roll_inner_PID,
      droll_setpoint - imu_data_.eulerRate[0]);
    u_euler_[0] = saturateFilter(u_euler_[0], -max_control_effort_[0], max_control_effort_[0]);

    /************************************************************************
    *                         Pitch Controller                              *
    ************************************************************************/
    float dpitch_setpoint;
    if (flight_mode_ == 1) {  // Stabilized Flight Mode
      dpitch_setpoint = update_filter(flight_filters_->pitch_outer_PID, setpoint_.euler_ref[1]
        - imu_data_.euler[1]);
    } else if (flight_mode_ == 2) {  // Acro mode
      dpitch_setpoint = setpoint_.euler_ref[1];
    }

    imu_data_.eulerRate[1] = update_filter(flight_filters_->gyro_lpf[1], imu_data_.eulerRate[1]);
    u_euler_[1] = update_filter(flight_filters_->pitch_inner_PID,
      dpitch_setpoint - imu_data_.eulerRate[1]);
    u_euler_[1] = saturateFilter(u_euler_[1], -max_control_effort_[1], max_control_effort_[1]);

    /************************************************************************
    *                          Yaw Controller                              *
    ************************************************************************/
    imu_data_.eulerRate[2] = update_filter(flight_filters_->gyro_lpf[2], imu_data_.eulerRate[2]);
    u_euler_[2] = update_filter(flight_filters_->yaw_PID,
      setpoint_.euler_ref[2] - imu_data_.euler[2]);
    //Apply a saturation filter
    u_euler_[2] = saturateFilter(u_euler_[2], -max_control_effort_[2],
      max_control_effort_[2]);

    /************************************************************************
    *                  Reset the Integrators if Landed                      *
    ************************************************************************/
    if (setpoint_.throttle < setpoint_module_->GetMinThrottle() + .01) {
      integrator_reset_++;
    } else {
      integrator_reset_ = 0;
    }

    // if landed for 4 seconds, reset integrators and Yaw error
    if (integrator_reset_ > 2 / delta_t_) {
      setpoint_.euler_ref[2]=imu_data_.euler[2];
      setpoint_module_->SetYawRef(imu_data_.euler[2]);
      flight_filters_->ZeroPIDs();
    }

    /************************************************************************
    *  Mixing
    *
    *                          CCW 1    2 CW       Body Frame:
    *                               \ /               X
    *                               / \             Y_|
    *                           CW 3   4 CCW              Z UP
    *
    ************************************************************************/
    u_[0] = setpoint_.throttle + u_euler_[0] - u_euler_[1] - u_euler_[2];
    u_[1] = setpoint_.throttle - u_euler_[0] - u_euler_[1] + u_euler_[2];
    u_[2] = setpoint_.throttle + u_euler_[0] + u_euler_[1] + u_euler_[2];
    u_[3] = setpoint_.throttle - u_euler_[0] + u_euler_[1] - u_euler_[2];

    /************************************************************************
    *             Check Output Ranges, if outside, adjust                 *
    ************************************************************************/
    CheckOutputRange(u_);

    /************************************************************************
    *                  Send Commands to ESCs                         *
    ************************************************************************/
    if (!is_debug_mode_) {
      pru_client_.setSendData(u_);
    }

    /************************************************************************
    *             Check the kill Switch and Shutdown if set               *
    ************************************************************************/
    if (setpoint_.kill_switch[0] < 0.5 && !is_debug_mode_) {
      spdlog::info("\nKill Switch Hit! Shutting Down\n");
      rc_set_state(EXITING);
    }

    //Print some stuff to the console in debug mode
    if (is_debug_mode_) {
      ConsolePrint();
    }
    /************************************************************************
    *             Check for GPS Data and Handle Accordingly               *
    ************************************************************************/
    gps_module_.getGpsData(&gps_);

    /************************************************************************
    *               Log Important Flight Data For Analysis              *
    ************************************************************************/
    struct ULogFlightMsg flight_msg(GetTimeMicroseconds(), imu_data_, setpoint_, u_,
      u_euler_);
    ulog_.WriteFlightData<struct ULogFlightMsg>(flight_msg, ULogFlightMsg::ID());

    uint64_t timeFinish = GetTimeMicroseconds();
    uint64_t sleep_time = static_cast<uint64_t>(delta_t_*1.0E6) - (timeFinish - timeStart);

    // Check to make sure the elapsed time wasn't greater than time allowed.
    // If so don't sleep at all
    if (sleep_time < static_cast<uint64_t>(delta_t_*1.0E6)) {
      rc_usleep(sleep_time);
    } else {
      spdlog::warn("[flyMS] Error! Control thread too slow! time in micro seconds: {}",
        (timeFinish - timeStart));
    }
  }
  return 0;
}

uint64_t flyMS::GetTimeMicroseconds() {
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  // return (uint64_t)tv.tv_sec * 1E6 + (uint64_t)tv.tv_nsec / 1E3;
}


int flyMS::ConsolePrint() {
//  spdlog::info("time {:3.3f} ", control->time);
//  spdlog::info("Alt_ref {:3.1f} ",control->alt_ref);
  // spdlog::info("U1: {:2.2f}, U2: {:2.2f}, U3: {:2.2f}, U4: {:2.2f} ", u_[0], u_[1], u_[2], u_[3]);
 // spdlog::info("Aux {:2.1f} ", setpoint_.Aux[0]);
//  spdlog::info("function: {}",rc_get_dsm_ch_normalized(6));
//  spdlog::info("num wraps {} ",control->num_wraps);
  spdlog::info(" Throt {:2.2f}, Roll_ref {:2.2f}, Pitch_ref {:2.2f}, Yaw_ref {:2.2f} ",
    setpoint_.throttle, setpoint_.euler_ref[0], setpoint_.euler_ref[1], setpoint_.euler_ref[2]);
  spdlog::info("Roll {:1.2f}, Pitch {:1.2f}, Yaw {:2.3f}", imu_data_.euler[0],
    imu_data_.euler[1], imu_data_.euler[2]);
//  spdlog::info(" Mag X {:4.2f}",control->mag[0]);
//  spdlog::info(" Mag Y {:4.2f}",control->mag[1]);
//  spdlog::info(" Mag Z {:4.2f}",control->mag[2]);
  // spdlog::info(" Accel X {:4.2f}",control->accel[0]);
  // spdlog::info(" Accel Y {:4.2f}",control->accel[1]);
  // spdlog::info(" Accel Z {:4.2f}",control->accel[2]);
//   spdlog::info(" Pos N {:2.3f} ", control->ekf_filter.output.ned_pos[0]);
//  spdlog::info(" Pos E {:2.3f} ", control->ekf_filter.output.ned_pos[1]);
//  spdlog::info(" Pos D {:2.3f} ", control->ekf_filter.output.ned_pos[2]);
  // spdlog::info(" DRoll {:1.2f}, DPitch {:1.2f}, DYaw {:2.3f}", imu_data_.eulerRate[0],
  //   imu_data_.eulerRate[1], imu_data_.eulerRate[2]);
  // spdlog::info("uroll {:2.3f}, upitch {:2.3f}, uyaw {:2.3f}", u_euler_[0], u_euler_[1],
  //   u_euler_[2]);
//  spdlog::info(" GPS pos lat: {:2.2f}", control->GPS_data.pos_lat);
//  spdlog::info(" GPS pos lon: {:2.2f}", control->GPS_data.pos_lon);
//  spdlog::info(" HDOP: {}", control->GPS_data.HDOP);
//  spdlog::info("Baro Alt: {} ",control->baro_alt);
  return 0;
}

int flyMS::CheckOutputRange(std::array<float, 4> &u) {
  float largest_value = 1;
  float smallest_value = 0;

  for (int i = 0; i < 4; i++) {
    if (u[i] > largest_value) largest_value = u[i];

    if (u[i] < smallest_value) u[i] = 0;
  }

  // if upper saturation would have occurred, reduce all outputs evenly
  if (largest_value > 1) {
    float offset = largest_value - 1;
    for (int i = 0; i < 4; i++) u[i] -= offset;
  }
  return 0;
}

std::string flyMS::GetLogDir(const std::string &log_location) {
  int run_number = 1;
  std::stringstream run_str;
  run_str << std::internal << std::setfill('0') << std::setw(3) << run_number;

  std::string run_folder(log_location + std::string("/run") + run_str.str());

  //Find the next run number folder that isn't in use
  struct stat st = {0};
  while (!stat(run_folder.c_str(), &st)) {
    run_str.str(std::string());
    run_str << std::internal << std::setfill('0') << std::setw(3) << ++run_number;
    run_folder = (log_location + std::string("/run") + run_str.str());
  }

  //Make a new folder to hold the logged data
  mkdir(run_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  return run_folder;
}

void flyMS::InitializeSpdlog(const std::string &log_dir) {
  int max_bytes = 1048576 * 20;  // Max 20 MB
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
  flyMS_log->flush_on(spdlog::level::critical);
}

int flyMS::StartupRoutine() {
  // Create a file for logging and initialize our file logger
  std::string log_dir = GetLogDir(log_filepath_);
  InitializeSpdlog(log_dir);
  ulog_.InitUlog(log_dir);

  //Initialize the remote controller through the setpoint object
  setpoint_module_ = std::make_unique<Setpoint>(config_params_);


  //Tell the system that we are running
  rc_set_state(RUNNING);

  // Initialize the PID controllers and LP filters
    YAML::Node controller = config_params_["controller"];
    YAML::Node filters = config_params_["filters"];
    flight_filters_ = std::make_unique<FlightFilters> (
      controller["roll_PID_inner"].as<std::array<float, 3> >(),
      controller["roll_PID_outer"].as<std::array<float, 3> >(),
      controller["pitch_PID_inner"].as<std::array<float, 3> >(),
      controller["pitch_PID_outer"].as<std::array<float, 3> >(),
      controller["yaw_PID"].as<std::array<float, 3> >(),
      filters["imu_lpf_num"].as<std::vector<float> >(),
      filters["imu_lpf_den"].as<std::vector<float> >(),
      delta_t_,
      controller["pid_LPF_const_sec"].as<float>());

  mavlink_interface_.Init();

  // Initialize the IMU Hardware
  imu_module_ = std::make_unique<ImuDmp>(config_params_["imu_params"]);

  // Initialize the client to connect to the PRU handler
  pru_client_.startPruClient();

  // Start the flight program
  flightcore_thread_ = std::thread(&flyMS::FlightCore, this);

  // Start the flight program
  sched_param sch_params;
  sch_params.sched_priority = 1;  // Max Priority
  if(pthread_setschedparam(flightcore_thread_.native_handle(), SCHED_FIFO, &sch_params)) {
    perror("error with pthread");
    spdlog::error("Error setting pthread_setschedparam");
  }
  return 0;
}
