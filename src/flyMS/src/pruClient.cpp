/**
 * @file pruClient.hpp
 * @brief Source code to talk to pruClient module for ESC control signals
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */


#include "flyMS/pruClient.hpp"

pruClient::pruClient() : u(NUM_CHANNELS) {

}

pruClient::~pruClient() {
  //Join the thread if executing
  if (this->pruSenderThread.joinable())
    this->pruSenderThread.join();
}

int pruClient::startPruClient() {
  //Check to see if the pru server is runningi, if not start it
  if (access(PRU_PID_FILE, F_OK) == -1) {
    printf("Pru handler is not runnining, starting a new process now");
    system("nohup pru_handler >/dev/null 2>&1 &");
    //Give it some time to initialize
    sleep(1);

    //Check again to make sure it's on
    if (access(PRU_PID_FILE, F_OK) == -1) {
      printf("Error! pru_handler will not start\n");
      return -1;
    }
  }

  //Initialize the network socket on localhost
  if ((this->sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    std::cout << "\n Error : Could not create socket: " << strerror(errno) << std::endl;
    return -1;
  }

  this->serv_addr.sin_family = AF_INET;
  this->serv_addr.sin_port = htons(5000);

  // if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
  if (inet_pton(AF_INET, "127.0.0.1" , &this->serv_addr.sin_addr) <= 0) {
    std::cout << "\n  inet_pton error occured: " << strerror(errno) << std::endl;
    return -1;
  }

  //Connect to the socket
  if (connect(this->sockfd, (struct sockaddr *)&this->serv_addr, sizeof(this->serv_addr)) < 0) {
    std::cout << "\n Error : Connect Failed: " << strerror(errno) << std::endl;
    return -1;
  }

  this->pruSenderThread = std::thread(&pruClient::pruSender, this);
  return 0;
}

int pruClient::setSendData(std::vector<float> _u) {
  //Gain access to shared memory
  this->pruSenderMutex.lock();

  if (_u.size() != NUM_CHANNELS)
    std::cout << "[pruClient]! Wrong number of channels given" << std::endl;
  //Set the values
  this->u = _u;

  this->send_flag = true;

  //Give shared memory access back
  this->pruSenderMutex.unlock();
  return 0;
}

int pruClient::pruSender() {

  uint16_t tmp16 = 0x0000;
  int i;
  while (rc_get_state() != EXITING) {
    //Gain access to shared memory
    this->pruSenderMutex.lock();

    if (this->send_flag) {
      this->sendBuff[0] = 0xAA;
      this->sendBuff[1] = 0xBB;
      this->sendBuff[10] = 0xEE;
      this->sendBuff[11] = 0xFF;

      for (i = 0; i < NUM_CHANNELS ; i++) {
        //Apply a saturation filter spanning 0 to 1
        if (this->u[i] < 0.0f) this->u[i] = 0.0f;
        else if (this->u[i] > 1.0f) this->u[i] = 1.0f;

        //Convert the float to the uint16 send array
        if (this->u[i] == 0.0f) tmp16 = 0;
        else tmp16 = (uint16_t)(this->u[i] * 65536.0f) - 1;

        this->sendBuff[2 * i + 2] = tmp16 >> 8;
        this->sendBuff[2 * i + 3] = tmp16 & 0xFF;
      }
      //Send the data
      write(this->sockfd, this->sendBuff, sizeof(this->sendBuff) - 1);
      this->send_flag = false;
    }

    //Give shared memory access back
    this->pruSenderMutex.unlock();

    usleep(2500); //run at 400hz just to check
  }

  //Issue the shutdown command to the pru handler
  for (i = 0; i < 4; i++) {
    this->sendBuff[0] = 0xAA;
    this->sendBuff[1] = 0xBB;
    this->sendBuff[10] = 0xEE;
    this->sendBuff[11] = 0xFF;
    this->sendBuff[2] = 's';
    this->sendBuff[3] = 'h';
    this->sendBuff[4] = 'u';
    this->sendBuff[5] = 't';
    this->sendBuff[6] = 'd';
    this->sendBuff[7] = 'o';
    this->sendBuff[8] = 'w';
    this->sendBuff[9] = 'n';
    write(this->sockfd, this->sendBuff, sizeof(this->sendBuff) - 1);
  }
  return 0;
}
