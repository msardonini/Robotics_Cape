/**
 * @file pruClient.hpp
 * @brief Source code to talk to pruClient module for ESC control signals
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */


#include "flyMS/pruClient.h"

#include "spdlog/spdlog.h"
#include "unistd.h"

constexpr int NUM_CHANNELS = 4;

pruClient::pruClient() {}

pruClient::~pruClient() {
  //Issue the shutdown command to the pru handler
  char send_buffer[16];
  for (int i = 0; i < 4; i++) {
    send_buffer[0] = 0xAA;
    send_buffer[1] = 0xBB;
    send_buffer[10] = 0xEE;
    send_buffer[11] = 0xFF;
    send_buffer[2] = 's';
    send_buffer[3] = 'h';
    send_buffer[4] = 'u';
    send_buffer[5] = 't';
    send_buffer[6] = 'd';
    send_buffer[7] = 'o';
    send_buffer[8] = 'w';
    send_buffer[9] = 'n';
    write(sockfd_, send_buffer, sizeof(send_buffer) - 1);
  }
  close(sockfd_);
}

int pruClient::startPruClient() {
  //Check to see if the pru server is runningi, if not start it
  if (access(PRU_PID_FILE, F_OK) == -1) {
    spdlog::warn("Pru handler is not runnining, starting a new process now");
    system("nohup pru_handler >/dev/null 2>&1 &");
    //Give it some time to initialize
    sleep(1);

    //Check again to make sure it's on
    if (access(PRU_PID_FILE, F_OK) == -1) {
      spdlog::error("Error! pru_handler will not start\n");
      return -1;
    }
  }

  //Initialize the network socket on localhost
  if ((sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    spdlog::error("Error : Could not create socket: {}", strerror(errno));
    return -1;
  }

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(5000);

  // if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
  if (inet_pton(AF_INET, "127.0.0.1" , &serv_addr.sin_addr) <= 0) {
    spdlog::error("Error! inet_pton error occured: {}", strerror(errno));
    return -1;
  }

  //Connect to the socket
  if (connect(sockfd_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    spdlog::error("Error : Connect Failed: {}", strerror(errno));
    return -1;
  }

  return 0;
}

int pruClient::setSendData(std::array<float, NUM_CHANNELS> u) {
  char send_buffer[16];

  send_buffer[0] = 0xAA;
  send_buffer[1] = 0xBB;
  send_buffer[10] = 0xEE;
  send_buffer[11] = 0xFF;

  for (int i = 0; i < NUM_CHANNELS ; i++) {
    //Apply a saturation filter spanning 0 to 1
    if (u[i] < 0.0f) {
      u[i] = 0.0f;
    } else if (u[i] > 1.0f) {
      u[i] = 1.0f;
    }

    uint16_t tmp16 = 0x0000;
    //Convert the float to the uint16 send array
    if (u[i] == 0.0f) {
      tmp16 = 0;
    } else {
      tmp16 = (uint16_t)(u[i] * 65536.0f) - 1;
    }

    send_buffer[2 * i + 2] = tmp16 >> 8;
    send_buffer[2 * i + 3] = tmp16 & 0xFF;
  }
  //Send the data
  write(sockfd_, send_buffer, sizeof(send_buffer) - 1);

  return 0;
}
