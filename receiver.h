#ifndef RECEIVER_H
#define RECEIVER_H


#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

extern pthread_t ovp_rx_thread;

extern uint64_t ovp_refill_count;
extern uint64_t ovp_refill_error_count;
extern uint64_t ovp_forwarded_count;

int start_ovp_receiver(void);
void stop_ovp_receiver(void);

void receiver_ok_to_forward_frames(bool ok);

#endif // RECEIVER_H