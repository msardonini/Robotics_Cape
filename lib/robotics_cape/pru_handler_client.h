#ifndef PRUHANDLERCLINET_H
#define PRUHANDLERCLINET_H



#define PRU_PID_FILE "/var/run/pru_handler.pid"

typedef struct pru_client_data_t{
         uint8_t         send_flag;
         float           u[8];
 
}pru_client_data_t;

int start_pru_client(pru_client_data_t* pru_client_data);
void *pru_sender(void* ptr);
int join_pru_client();

#endif
