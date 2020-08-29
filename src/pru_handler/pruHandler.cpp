/**
 * @file pruHandler.cpp
 * @brief Process which communicates with the PRU that controlls quadrotor ESCs.
     This process connects to applications via TCP packets over localhost and
     relays the data to the PRU. If no program is active, zero throttle is sent.
 *
 * @author Mike Sardonini
 * @date 11/10/2018
 */

#include "pruHandler.hpp"

//Constructor
pruHandler::pruHandler() :
  pruState(PruState::UNINITIALIZED),
  listenfd(0),
  connfd(0),
  serv_addr(),
  rcvBuff(),
  logFid(LOG_FILE_PRU) {

}

pruHandler::~pruHandler() {
    close(connfd);
  remove(PID_FILE_PRU);
  this->logFid.close();

  if(this->pruHandlerThread.joinable())
    this->pruHandlerThread.join();
}

int pruHandler::init_pru_handler() {

  //Check to see if this process is running somewhere else and kill it
  this->checkForCompetingProcess();

  // create new pid file
  this->createPIDFile();

  //Initialize the servo connectors from the Robotics Cape Library
  if(rc_servo_init() == -1) {
    return -1;
  } 

  //Initialize the server to receive commands to send to ESCs
  this->initServer();

  this->pruState = PruState::RUNNING;

  this->pruHandlerThread = std::thread(&pruHandler::run, this);
  return 0;
}


int pruHandler::checkForCompetingProcess() {
  FILE* fd;
  int old_pid, i;
  // start by checking if a pid file exists
  if(access(PID_FILE_PRU, F_OK ) != 0){
    // PID file missing
    return 0;
  }
  // attempt to open PID file
  // if the file didn't open, no project is runnning in the background
  // so return 0
  fd = fopen(PID_FILE_PRU, "r");
  if(fd==NULL) return 0;
  // try to read the current process ID
  fscanf(fd,"%d", &old_pid);
  fclose(fd);
  // if the file didn't contain a PID number, remove it and
  // return -1 indicating weird behavior
  if(old_pid == 0){
    remove(PID_FILE_PRU);
    return -2;
  }
  // check if it's our own pid, if so return 0
  if(old_pid == (int)getpid()) return 0;
  // now see if the process for the read pid is still running
  if(getpgid(old_pid) < 0){
    // process not running, remove the pid file
    remove(PID_FILE_PRU);
    return 0;
  }
  // process must be running, attempt a clean shutdown
  kill((pid_t)old_pid, SIGINT);
  // check every 0.1 seconds to see if it closed
  for(i=0; i<30; i++){
    if(getpgid(old_pid) >= 0) rc_usleep(100000);
    else{ // succcess, it shut down properly
      remove(PID_FILE_PRU);
      return 1;
    }
  }
  // otherwise force kill the program if the PID file never got cleaned up
  kill((pid_t)old_pid, SIGKILL);
  rc_usleep(500000);
  // delete the old PID file if it was left over
  remove(PID_FILE_PRU);
  // return -1 indicating the program had to be killed
  return -1;
}

int pruHandler::createPIDFile() {


  this->logFid << "[pruHandler] opening PID_FILE" << std::endl;

  FILE *fd;
  fd = fopen(PID_FILE_PRU, "ab+");
  if (fd == NULL) {
    this->logFid << "[pruHandler] error opening PID_FILE for writing" << std::endl;
    return -1;
  }
  pid_t current_pid = getpid();
  fprintf(fd,"%d",(int)current_pid);
  fflush(fd);
  fclose(fd);

  // Print current PID
  this->logFid << "[pruHandler] Process ID: "<<(int)current_pid<< std::endl;
  return 0;
}

int pruHandler::initServer() {
  this->listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&this->serv_addr, 0, sizeof(this->serv_addr));
    memset(this->rcvBuff, 0, sizeof(this->rcvBuff));

    this->serv_addr.sin_family = AF_INET;
    this->serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    this->serv_addr.sin_port = htons(PRU_PORT);

  if(bind(this->listenfd, (struct sockaddr*)&this->serv_addr, sizeof(this->serv_addr)) < 0)
  {
    this->logFid<< "[pruHandler] Bind Failed: " << strerror(errno) <<std::endl;
    return -1;
  }


  if(listen(this->listenfd, 10)) {
    this->logFid<< "[pruHandler] Listen Failed: " << strerror(errno) <<std::endl;
    return -1;
  }

  int flags = fcntl(this->listenfd, F_GETFL, 0);
  fcntl(this->listenfd, F_SETFL, flags | O_NONBLOCK);

  return 0;
}


int pruHandler::run() {
  float val[4];
  int i = 0, n = 0;

  struct timeval timeoutStandby;
  struct timeval timeoutRunning;
  fd_set set;
  int rv;

  start_over:
  while (this->pruState != PruState::EXITING) {
    FD_ZERO(&set); /* clear the set */
    FD_SET(this->listenfd, &set); /* add our file descriptor to the set */

    timeoutStandby.tv_sec = 0;
    timeoutStandby.tv_usec = 20000; //Set 20 ms timeout

    rv = select(this->listenfd + 1, &set, NULL, NULL, &timeoutStandby);
    if (rv == 0)
    {
#ifdef DEBUG
    //  this->logFid<< "Timeout Detected!" << std::endl;
#endif
    }
    if (rv > 0)
    {
      this->connfd = accept(this->listenfd, (struct sockaddr*)NULL, NULL);
      if (this->connfd >=0)
      {
        this->logFid<< "Connection Received!" << std::endl;
        break;
      }
    }

    //Send zero throttle when not connected to a program
    for (i = 0; i < PRU_NUM_CHANNELS; i++)
      rc_servo_send_esc_pulse_normalized(i+1, 0.0);
  }

  timeoutRunning.tv_sec = 0;
  timeoutRunning.tv_usec = 30000;  // Set 30 ms timeout
  setsockopt(this->connfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeoutRunning,
    sizeof(struct timeval));
  while (this->pruState != PruState::EXITING) {
    n = read(this->connfd, rcvBuff, sizeof(rcvBuff)-1);
    if (n <= 0) {
      //#ifdef DEBUG
      //printf("Timeout detected!! \n");
      //this->logFid<< "Timeout Detected!" << std::endl;
      //#endif
      for (i = 0; i < PRU_NUM_CHANNELS; i++)
        rc_servo_send_esc_pulse_normalized(i+1, 0.0);
    } else {
      this->rcvBuff[n] = 0;


      //Only proceed if the start/end bytes come in as expected
      if (this->rcvBuff[0]  == 0xAA && this->rcvBuff[1] == 0xBB
        &&  this->rcvBuff[10] == 0xEE && this->rcvBuff[11] == 0xFF) {
        //Check for shutdown command
        if (  this->rcvBuff[2] == 's'
          &&  this->rcvBuff[3] == 'h'
          &&  this->rcvBuff[4] == 'u'
          &&  this->rcvBuff[5] == 't'
          &&  this->rcvBuff[6] == 'd'
          &&  this->rcvBuff[7] == 'o'
          &&  this->rcvBuff[8] == 'w'
          &&  this->rcvBuff[9] == 'n')
        {
          close(this->connfd);
          this->logFid<< "Closing Session!" << std::endl;
          goto start_over;
        } else {
        //printf("Sending Values: ");
          for (i = 0; i < 4; i++) {
            val[i] = ((float)((this->rcvBuff[2*i+2] << 8) + this->rcvBuff[2*i+3]))/65536.0f;
            rc_servo_send_esc_pulse_normalized(i+1,val[i]);
          //  printf(" %f, ", val[i]);
          }
        //printf("\n");
        }
      }
    }
  }
  return 0;
}

void pruHandler::shutdownPruHandler(int signo) {
  switch(signo) {
    case SIGINT: // normal ctrl-c shutdown interrupt
      this->pruState = PruState::EXITING;
      this->logFid<< "[pruHandler] received SIGINT Ctrl-C" << std::endl;
      break;
    case SIGTERM: // catchable terminate signal
      this->pruState = PruState::EXITING;
      this->logFid<< "[pruHandler] received SIGTERM" << std::endl;
      break;
    case SIGHUP: // terminal closed or disconnected, carry on anyway
      break;
    default:
      this->logFid<< "[pruHandler] Warning! Received unknown signal" << std::endl;
      break;
  }
  return;
 }





 // ------------------------------- END C++ VERSION --------------------------------------- //




// void shutdown_pru_handler(int signo);

// void shutdown_pru_handler(int signo){
//          switch(signo){
//          case SIGINT: // normal ctrl-c shutdown interrupt
//                 pru_set_state(PRUEXITING);
//         #ifdef DEBUG
//                 printf("\nreceived SIGINT Ctrl-C\n");
//                 #endif
//         break;
//          case SIGTERM: // catchable terminate signal
//                 pru_set_state(PRUEXITING);
//         #ifdef DEBUG
//                 printf("\nreceived SIGTERM\n");
//                 #endif
//         break;
//          case SIGHUP: // terminal closed or disconnected, carry on anyway
//                  break;
//          default:
//                  break;
//          }
//          return;
//  }


// void pru_disable_signal_handler(){
//          signal(SIGINT, shutdown_pru_handler);
//          signal(SIGKILL, shutdown_pru_handler);
//          signal(SIGHUP, shutdown_pru_handler);
//          return;
// }



// int rc_kill_pru(){
//   FILE* fd;
//   int old_pid, i;
//   // start by checking if a pid file exists
//   if(access(PID_FILE_PRU, F_OK ) != 0){
//     // PID file missing
//     return 0;
//   }
//   // attempt to open PID file
//   // if the file didn't open, no project is runnning in the background
//   // so return 0
//   fd = fopen(PID_FILE_PRU, "r");
//   if(fd==NULL) return 0;
//   // try to read the current process ID
//   fscanf(fd,"%d", &old_pid);
//   fclose(fd);
//   // if the file didn't contain a PID number, remove it and
//   // return -1 indicating weird behavior
//   if(old_pid == 0){
//     remove(PID_FILE_PRU);
//     return -2;
//   }
//   // check if it's our own pid, if so return 0
//   if(old_pid == (int)getpid()) return 0;
//   // now see if the process for the read pid is still running
//   if(getpgid(old_pid) < 0){
//     // process not running, remove the pid file
//     remove(PID_FILE_PRU);
//     return 0;
//   }
//   // process must be running, attempt a clean shutdown
//   kill((pid_t)old_pid, SIGINT);
//   // check every 0.1 seconds to see if it closed
//   for(i=0; i<30; i++){
//     if(getpgid(old_pid) >= 0) rc_usleep(100000);
//     else{ // succcess, it shut down properly
//       remove(PID_FILE_PRU);
//       return 1;
//     }
//   }
//   // otherwise force kill the program if the PID file never got cleaned up
//   kill((pid_t)old_pid, SIGKILL);
//   rc_usleep(500000);
//   // delete the old PID file if it was left over
//   remove(PID_FILE_PRU);
//   // return -1 indicating the program had to be killed
//   return -1;
// }


// int init_pru_handler()
// {
//   //Check to see if this process is running somewhere else
//   rc_kill_pru();

//   //startup the signal handler
//   pru_disable_signal_handler();

//   pru_set_state(RUNNING);

//   // start PRU
//   #ifdef DEBUG
//   printf("Initializing: PRU\n");
//   #endif
//   initialize_pru();

//   // create new pid file with process id
//   #ifdef DEBUG
//     printf("opening PID_FILE\n");
//   #endif
//   FILE *fd;
//   fd = fopen(PID_FILE_PRU, "ab+");
//   if (fd == NULL) {
//     fprintf(stderr,"error opening PID_FILE for writing\n");
//     return -1;
//   }
//   pid_t current_pid = getpid();
//   fprintf(fd,"%d",(int)current_pid);
//   fflush(fd);
//   fclose(fd);

//   // Print current PID
//   #ifdef DEBUG
//   printf("Process ID: %d\n", (int)current_pid);
//    #endif

//   restart_pru();

//   return 0;
// }


// //#define DEBUG

// int main(int argc, char *argv[])
// {

//   float val[4];
//   int i = 0, n = 0;

//   struct timeval tv;
//   fd_set set;
//   struct timeval timeout;
//   int rv;


//   start_over:
//   while(pru_get_state()!=PRUEXITING)
//   {
//     FD_ZERO(&set); /* clear the set */
//     FD_SET(this->listenfd, &set); /* add our file descriptor to the set */

//     timeout.tv_sec = 0;
//     timeout.tv_usec = 20000; //Set 20 ms timeout

//     rv = select(this->listenfd + 1, &set, NULL, NULL, &timeout);
//     if (rv == 0)
//     {
//       #ifdef DEBUG
//       printf("timeout detected\n");
//       #endif
//     }
//     if (rv > 0)
//     {
//       this->connfd = accept(this->listenfd, (struct sockaddr*)NULL, NULL);
//       if (this->connfd >=0)
//       {
//         #ifdef DEBUG
//         printf("Connection Received!\n");
//         #endif
//         break;
//       }
//     }
//     rc_send_esc_pulse_normalized_all(0);
//   }

//   tv.tv_sec = 0;
//   tv.tv_usec = 30000;  // Set 30 ms timeout
//   setsockopt(this->connfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval));
//   while(pru_get_state()!=PRUEXITING)
//     {
//     n = read(this->connfd, rcvBuff, sizeof(rcvBuff)-1);
//     if (n <= 0)
//     {
//       //#ifdef DEBUG
//       //printf("Timeout detected!! \n");
//       //#endif
//       rc_send_esc_pulse_normalized_all(0.0);
//     }
//     else
//     {
//       this->rcvBuff[n] = 0;


//       //Only proceed if the start/end bytes come in as expected
//       if (this->rcvBuff[0]  == 0xAA && this->rcvBuff[1] == 0xBB
//         &&  this->rcvBuff[10] == 0xEE && this->rcvBuff[11] == 0xFF)
//       {
//         //Check for shutdown command
//         if (  this->rcvBuff[2] == 's'
//           &&  this->rcvBuff[3] == 'h'
//           &&  this->rcvBuff[4] == 'u'
//           &&  this->rcvBuff[5] == 't'
//           &&  this->rcvBuff[6] == 'd'
//           &&  this->rcvBuff[7] == 'o'
//           &&  this->rcvBuff[8] == 'w'
//           &&  this->rcvBuff[9] == 'n')
//         {
//           close(this->connfd);
//           #ifdef DEBUG
//           printf("Closing session! \n");
//           #endif
//           goto start_over;
//         }
//         else
//         {
//         //printf("Sending Values: ");
//           for (i = 0; i < 4; i++)
//           {
//             val[i] = ((float)((this->rcvBuff[2*i+2] << 8) + this->rcvBuff[2*i+3]))/65536.0f;
//             rc_send_esc_pulse_normalized(i+1,val[i]);
//           //  printf(" %f, ", val[i]);
//           }
//         //printf("\n");
//         }
//       }
//     }
//     }

//     close(connfd);
//   remove(PID_FILE_PRU);

//   return 0;
// }


