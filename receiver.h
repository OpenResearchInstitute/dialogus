
#include <pthread.h>



// OVP receiver thread
extern pthread_t ovp_rx_thread;


// Receiver thread
int start_ovp_receiver(void);
void stop_ovp_receiver(void);

void receiver_ok_to_forward_frames(bool ok);
