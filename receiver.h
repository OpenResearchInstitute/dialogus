#ifndef RECEIVER_H
#define RECEIVER_H


#include <pthread.h>

extern pthread_t ovp_rx_thread;

int start_ovp_receiver(void);
void stop_ovp_receiver(void);

void receiver_ok_to_forward_frames(bool ok);

#endif // RECEIVER_H