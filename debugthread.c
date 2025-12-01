#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>


#include "debugthread.h"
#include "registers.h"
#include "timestamp.h"

extern void print_rssi(void);

// OVP debug thread mechanisms
static pthread_t ovp_debug_thread;
extern bool stop;
extern pthread_mutex_t timeline_lock;


// ---------- Debug Thread -----------------------------

void* ovp_debug_thread_func(__attribute__((unused)) void *arg) {
	uint32_t sync_status __attribute__((unused));
	bool frame_sync_locked __attribute__((unused));
	bool frame_buffer_overflow __attribute__((unused));
	uint32_t frames_received __attribute__((unused)) = 0;
	uint32_t frame_sync_errors __attribute__((unused)) = 0;
	char *state_names[8] = { "IDLE", "COLLECT", "ENC_START", "ENC_RUN", "ENC_FLUSH", "INTERLEAVE", "OUTPUT", "iNvALiD" };

	while (!stop) {
		uint32_t now;

		pthread_mutex_lock(&timeline_lock);

			now = get_timestamp_ms();
			uint32_t tx_fifo_reg = capture_and_read_msk(OFFSET_MSK(tx_async_fifo_rd_wr_ptr));
			uint32_t rx_fifo_reg = capture_and_read_msk(OFFSET_MSK(rx_async_fifo_rd_wr_ptr));
//			if (rx_fifo_reg == 0xDEADBEEF) {
//				printf("debugthread fifo at %d ms can't read RX FIFO right now\n", now);
//			}
//			if (tx_fifo_reg == 0xDEADBEEF) {
//				printf("debugthread fifo at %d ms can't read TX FIFO right now\n", now);
//			} else {
//				printf("debugthread fifo at %d ms: tx-w:%03x tx-r:%03x rx-w:%03x rx-r:%03x tx-debug:%02x\n", now,
//								(tx_fifo_reg & 0x03FF0000) >> 16,
//								(tx_fifo_reg & 0x00000FFF),
//								(rx_fifo_reg & 0x03FF0000) >> 16,
//								(rx_fifo_reg & 0x00000FFF),
//								(tx_fifo_reg & 0xFC000000) >> 26
//							);
//				printf("encoder_tvalid = %d\n", (tx_fifo_reg & 0x80000000) ? 1 : 0); 
//				printf("encoder_tready = %d\n", (tx_fifo_reg & 0x40000000) ? 1 : 0); 
//				printf("   fifo_tvalid = %d\n", (tx_fifo_reg & 0x20000000) ? 1 : 0); 
//				printf(".  fifo_tready = %d\n", (tx_fifo_reg & 0x10000000) ? 1 : 0); 
//				printf("        tx_req = %d\n", (tx_fifo_reg & 0x08000000) ? 1 : 0); 
//				printf(" encoder_tlast = %d\n", (tx_fifo_reg & 0x04000000) ? 1 : 0); 
//			}
			// VRVR QLxx xwww wwww wwws ssrr rrrr rrrr

//			printf("DEBUG: %02x %s wr=%03x rd=%03x\n", (tx_fifo_reg & 0xfc000000) >> 26,
//													   state_names[(tx_fifo_reg & 0x03800000) >> 23],
//													   (tx_fifo_reg & 0x007FE000) >> 13,
//													   (tx_fifo_reg & 0x000003FF)
//						);
			printf("DEBUG: etv=%x etr=%x ftv=%x ftr=%x txr=%x etl=%x %s wr=%03x rd=%03x rxwr=%03x rxrd=%03x\n", 
						(tx_fifo_reg & 0x80000000) >> 31,
						(tx_fifo_reg & 0x40000000) >> 30,
						(tx_fifo_reg & 0x20000000) >> 29,
						(tx_fifo_reg & 0x10000000) >> 28,
						(tx_fifo_reg & 0x08000000) >> 27,
						(tx_fifo_reg & 0x04000000) >> 26,
						state_names[(tx_fifo_reg & 0x03800000) >> 23],
						(tx_fifo_reg & 0x007FE000) >> 13,
						(tx_fifo_reg & 0x000003FF),

						(rx_fifo_reg & 0x03FF0000) >> 16,
						(rx_fifo_reg & 0x000003FF)
						);
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
//			printf("debugthread axis at %d ms: axis_xfer_count = 0x%08x\n", now, 
//					capture_and_read_msk(OFFSET_MSK(axis_xfer_count)));

		pthread_mutex_unlock(&timeline_lock);

		usleep(100);
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

