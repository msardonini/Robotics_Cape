/**
 * @file gps.cpp
 * @brief Source code to communicate with the trimble copernicus II GPS module
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "flyMS/gps.hpp"

#include "spdlog/spdlog.h"

//TODO take our the hardware inits outside of the class constructor
//Default Constructor
gps::gps(const YAML::Node input_params) {
  int res;
  struct termios newtio;
  char buf[255];

  this->serialFd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
  if (this->serialFd < 0) { perror(MODEMDEVICE); exit(-1); }

  bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

  /* BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
     CRTSCTS : output hardware flow control (only used if the cable has
               all necessary lines. See sect. 7 of Serial-HOWTO)
     CS8     : 8n1 (8bit,no parity,1 stopbit)
     CLOCAL  : local connection, no modem contol
     CREAD   : enable receiving characters */
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

  /* IGNPAR  : ignore bytes with parity errors
     otherwise make device raw (no other input processing) */
  newtio.c_iflag = IGNPAR;

  /*  Raw output  */
  newtio.c_oflag = 0;

  /* ICANON  : enable canonical input
     disable all echo functionality, and don't send signals to calling program */
  newtio.c_lflag = ICANON;
  /* now clean the modem line and activate the settings for the port */
  tcflush(this->serialFd, TCIFLUSH);
  tcsetattr(this->serialFd, TCSANOW, &newtio);

  // Initialize file descriptor sets
  fd_set read_fds, write_fds, except_fds;
  FD_ZERO(&read_fds);
  FD_ZERO(&write_fds);
  FD_ZERO(&except_fds);
  FD_SET(this->serialFd, &read_fds);

  // Set timeout to 1.0 seconds
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  // Wait for input to become ready or until the time out; the first parameter is
  // 1 more than the largest file descriptor in any of the sets
  if (select(this->serialFd + 1, &read_fds, &write_fds, &except_fds, &timeout) == 1) {

    spdlog::info("\n Changing Baud to 57600. Look for >>$PTNLRPT,A*3D<< for successful transmission\n");
    write(this->serialFd, "$PTNLSPT,057600,8,N,1,4,4*12\r\n", 30); //Change Baud Rade to 57600
    usleep(300000);
    res = read(this->serialFd, buf, 255);
    buf[res] = 0;
    //spdlog::info("No Fix, Message transmitted from Module is: %s",buf);

  } else {
    // timeout or error
    //spdlog::info("Successfully taken off of 9800 Baud, Proceeding,\n");
  }

  //Now the device is reading faster at 57600 Baud, change settings on the beagleboard
  bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
  newtio.c_cflag = BAUDRATE2 | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = ICANON;
  tcflush(this->serialFd, TCIFLUSH);
  tcsetattr(this->serialFd, TCSANOW, &newtio);

  struct timeval timeout2;
  timeout2.tv_sec = 1;
  timeout2.tv_usec = 0;
  /*** Make Sure that the GPS Unit is Reading, if not Disable *******/
  fd_set read_fds2, write_fds2, except_fds2;
  FD_ZERO(&read_fds2);
  FD_ZERO(&write_fds2);
  FD_ZERO(&except_fds2);
  FD_SET(this->serialFd, &read_fds2);

  if (select(this->serialFd + 1, &read_fds2, &write_fds2, &except_fds2, &timeout2) == 1) {
    spdlog::info("GPS init successful\n");
    write(this->serialFd, "$PTNLSNM,0005,01*52\r\n", 21); //NMEA message to output GGA  & VTG only
    usleep(50000);
    //res = read(this->serialFd, buf, 255);
    //buf[res] = 0;
    //spdlog::info("%s", buf);

    write(this->serialFd, "$PTNLQBA*54\r\n", 13); //antenna check
    //spdlog::info("Antenna query\n");
    usleep(50000);
    //res = read(this->serialFd, buf, 255);
    //buf[res] = 0;
    //spdlog::info("%s", buf);

    //Start the GPS thread
    this->gpsThread = std::thread(&gps::dataMonitor, this);
    //spdlog::info("GPS Thread Started\n");
  } else {
    // timeout or error
    spdlog::info("GPS Failed, Contining on without GPS\n");
  }

}

//Default Destructor
gps::~gps() {
  //Join the Thread if running
  if (this->gpsThread.joinable())
    this->gpsThread.join();
}




int gps::dataMonitor() {
  char buf[255];
  int res, comma[15];
  char Deg_Lat_buf[12], Min_Lat_buf[12], Deg_Lon_buf[12], Min_Lon_buf[12];
  while (rc_get_state() != EXITING) {
    memset(buf, 0, sizeof(buf));


    res = read(this->serialFd, buf, 255);
    buf[res] = 0;
    //spdlog::info("%s\n",buf);
    //Clear all the buffers before reading any data
    memset(Deg_Lat_buf, 0, sizeof(Deg_Lat_buf)); memset(Min_Lat_buf, 0, sizeof(Min_Lat_buf));
    memset(Deg_Lon_buf, 0, sizeof(Deg_Lon_buf)); memset(Min_Lon_buf, 0, sizeof(Min_Lon_buf));
    memset(comma, 0, sizeof(comma));
    int count = 0, i = 0;
    if (buf[3] == 'G') if (buf[4] == 'G') if (buf[5] == 'A') {
          while (count < 13) {
            if (buf[i] == ',') {
              comma[count] = i;
              count++;
            }
            i++;
          }

          this->gpsMutex.lock();

          if ((buf[comma[5] + 1] == '1') || (buf[comma[5] + 1] == '2')) {
            this->gpsData.GPS_fix = 1;
          } else {
            this->gpsData.GPS_fix = 0;
          }

          //Parse Latitude
          for (i = comma[1]; i <= comma[2] - 2; i++) {
            if (i <= comma[1] + 1) {
              Deg_Lat_buf[i - comma[1]] = buf[i + 1];
            } else {
              Min_Lat_buf[i - comma[1] - 2] = buf[i + 1];
            }
          }
          //Parse Longitude
          for (i = comma[3]; i <= comma[4] - 2; i++) {
            if (i <= comma[3] + 2) {
              Deg_Lon_buf[i - comma[3]] = buf[i + 1];
            } else {
              Min_Lon_buf[i - comma[3] - 3] = buf[i + 1];
            }
          }

          //Parse Horizontal Dilution of Presision, accuracy of measurements
          this->gpsData.HDOP = get_NMEA_field(8, buf, comma);

          //Get altitude
          this->gpsData.gps_altitude = get_NMEA_field(9, buf, comma);

          //Convert Strings to floats for use
          this->gpsData.deg_latitude = strtod(Deg_Lat_buf, NULL);
          this->gpsData.deg_longitude = strtod(Deg_Lon_buf, NULL);
          this->gpsData.min_latitude = strtod(Min_Lat_buf, NULL);
          this->gpsData.min_longitude = strtod(Min_Lon_buf, NULL);

          if (this->gpsData.min_latitude != 0) {

            this->gpsData.meters_lat = (this->gpsData.deg_latitude +
                                        this->gpsData.min_latitude / 60) * 111000;
            this->gpsData.meters_lon = (this->gpsData.deg_longitude +
                                        this->gpsData.min_longitude / 60) * 111000 *
                                       cos((this->gpsData.deg_latitude +
                                            this->gpsData.min_latitude / 60) * D2R_GPS);
            this->GGA_flag = true;
          }
          this->gpsMutex.unlock();
        }

    if (buf[3] == 'V') if (buf[4] == 'T') if (buf[5] == 'G') {
          while (count < 8) {
            if (buf[i] == ',') {
              comma[count] = i;
              count++;
            }
            i++;
          }
          this->gpsMutex.lock();

          //Get Speed from NMEA Message
          this->gpsData.speed = get_NMEA_field(6, buf, comma);
          //Get Direction from NMEA Message
          this->gpsData.direction = get_NMEA_field(1, buf, comma);


          this->VTG_flag = true;
          this->gpsMutex.unlock();
        }

    //  spdlog::info("GGA: %d VTG %d Data ready: %d\n\n",GGA_flag,VTG_flag,GPS_data_flag);
    if (this->GGA_flag && this->VTG_flag) this->GPS_data_flag.store(false); //Flag to note that GPS data is ready

  }

  return 0;
}

int gps::getGpsData(GPS_data_t *_data) {
  if (this->GPS_data_flag.load()) {
    if (this->gpsMutex.try_lock_for(std::chrono::milliseconds(5))) {
      memcpy(_data, &this->gpsData, sizeof(GPS_data_t));
      this->GPS_data_flag.store(false);
      this->GGA_flag = false;
      this->VTG_flag = false;
      this->gpsMutex.unlock();
      return true;
    }
  }
  return 0;
}

float gps::get_NMEA_field(int field, char buf[], int comma[]) {
  int i;
  char value_buf[10];
  memset(value_buf, 0, sizeof(value_buf));
  for (i = comma[field - 1]; i <= comma[field] - 2; i++) {
    value_buf[i - comma[field - 1]] = buf[i + 1];
  }
  return strtod(value_buf, NULL);
}

void gps::read_raw_gps(char *buf, GPS_data_t *GPS_data) {
  int res;
  res = read(this->serialFd, buf, 255);
  buf[res] = 0;
}

