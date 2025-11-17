#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>


#include "debugthread.h"
#include "registers.h"

extern uint32_t get_timestamp_ms(void);
extern void print_rssi(void);

// OVP debug thread mechanisms
static pthread_t ovp_debug_thread;
extern bool stop;
extern pthread_mutex_t timeline_lock;


// ---------- Debug Thread -----------------------------

void* ovp_debug_thread_func(__attribute__((unused)) void *arg) {
	uint32_t sync_status;
	bool frame_sync_locked;
	bool frame_buffer_overflow;
	uint32_t frames_received = 0;
	uint32_t frame_sync_errors = 0;

	while (!stop) {
		uint32_t now;

		pthread_mutex_lock(&timeline_lock);
			now = get_timestamp_ms();
			printf("debugthread fifo at %d ms: tx fifo: %08x rx fifo: %08x\n", now,
							capture_and_read_msk(OFFSET_MSK(tx_async_fifo_rd_wr_ptr)),
							capture_and_read_msk(OFFSET_MSK(rx_async_fifo_rd_wr_ptr)));
			printf("debugthread power at %d %d ", now,
							capture_and_read_msk(OFFSET_MSK(rx_power)));
			print_rssi();
			sync_status = capture_and_read_msk(OFFSET_MSK(rx_frame_sync_status));
			frame_sync_locked = sync_status & 0x00000001;
			frame_buffer_overflow = sync_status & 0x00000002;
			frames_received = (sync_status & 0x03fffffc) >> 2;
			frame_sync_errors = ((sync_status & 0xfc000000) >> 26) & 0x3f;
			printf("debugthread frame at %d ms: raw 0x%08x rcvd %d, errs %d %s %s\n", now,
				sync_status,
				frames_received,
				frame_sync_errors,
				frame_sync_locked ? "LOCKED" : "unlocked",
				frame_buffer_overflow ? "OVERFLOW" : "");
		pthread_mutex_unlock(&timeline_lock);

		usleep(10000);
	}
	printf("debug thread exiting\n");
	return NULL;
}

int start_debug_thread(void) {

	if (pthread_create(&ovp_debug_thread, NULL, ovp_debug_thread_func, NULL) != 0) {
		perror("OVP: Failed to create debug thread");
		return -1;
	}
	
	printf("OVP: debug thread started successfully\n");
	return 0;
}

void stop_debug_thread(void) {
	if (ovp_debug_thread) {
		pthread_cancel(ovp_debug_thread);
	}
		
	printf("OVP: debug thread stopped\n");
}

