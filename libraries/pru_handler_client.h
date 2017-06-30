#ifdef PRUHANDLERCLINET_H
#define PRUHANDLERCLINET_H



typedef struct pru_client_data_t{
         uint8_t         send_flag;
         float           u[4];
 
}pru_client_data_t;

void *pru_sender(void* ptr);

#endif
