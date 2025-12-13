#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>


#include "debug_printf.h"
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
	char *tx_state_names[8] = { "IDLE", "COLLECT", "RANDOMIZE", "PREP_FEC", "FEC_ENCODE", "INTERLEAVE", "OUTPUT", "iNvALiD" };
	char *rx_state_names[8] = { "IDLE", "COLLECT", "EXTRACT", "DEINTERLEAVE", "PREP_FEC_DECODE", "FEC_DECODE", "DERANDOMIZE", "OUTPUT" };


	while (!stop) {
		debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX timeline_lock: locking in debugthread\n");
		pthread_mutex_lock(&timeline_lock);
			debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX timeline_lock: acquired in debugthread\n");

			// now = get_timestamp_ms();
			uint32_t tx_fifo_reg = capture_and_read_msk(OFFSET_MSK(tx_async_fifo_rd_wr_ptr));
			uint32_t rx_fifo_reg = capture_and_read_msk(OFFSET_MSK(rx_async_fifo_rd_wr_ptr));
			if (rx_fifo_reg == 0xDEADBEEF) {
				debug_printf(LEVEL_INFO, DEBUG_FIFO, "debugthread fifo -- can't read RX FIFO right now\n");
			}
			if (tx_fifo_reg == 0xDEADBEEF) {
				debug_printf(LEVEL_INFO, DEBUG_FIFO, "debugthread fifo -- can't read TX FIFO right now\n");
			} else {
//				debug_printf(LEVEL_INFO, DEBUG_FIFO, "debugthread fifo: tx-w:%03x tx-r:%03x rx-w:%03x rx-r:%03x tx-debug:%02x\n",
//								(tx_fifo_reg & 0x03FF0000) >> 16,
//								(tx_fifo_reg & 0x00000FFF),
//								(rx_fifo_reg & 0x03FF0000) >> 16,
//								(rx_fifo_reg & 0x00000FFF),
//								(tx_fifo_reg & 0xFC000000) >> 26
//							);
//				debug_printf(LEVEL_INFO, DEBUG_FIFO, "encoder_tvalid = %d\n", (tx_fifo_reg & 0x80000000) ? 1 : 0); 
//				debug_printf(LEVEL_INFO, DEBUG_FIFO, "encoder_tready = %d\n", (tx_fifo_reg & 0x40000000) ? 1 : 0); 
//				debug_printf(LEVEL_INFO, DEBUG_FIFO, "   fifo_tvalid = %d\n", (tx_fifo_reg & 0x20000000) ? 1 : 0); 
//				debug_printf(LEVEL_INFO, DEBUG_FIFO, ".  fifo_tready = %d\n", (tx_fifo_reg & 0x10000000) ? 1 : 0); 
//				debug_printf(LEVEL_INFO, DEBUG_FIFO, "        tx_req = %d\n", (tx_fifo_reg & 0x08000000) ? 1 : 0); 
//				debug_printf(LEVEL_INFO, DEBUG_FIFO, " encoder_tlast = %d\n", (tx_fifo_reg & 0x04000000) ? 1 : 0); 
//			}
			// V R V R   Q L x x   x w w w   w w w w   w w w s   s s r r   r r r r   r r r r

//			debug_printf(LEVEL_INFO, DEBUG_FIFO, "DEBUG: %02x %s wr=%03x rd=%03x\n", (tx_fifo_reg & 0xfc000000) >> 26,
//													   tx_state_names[(tx_fifo_reg & 0x03800000) >> 23],
//													   (tx_fifo_reg & 0x007FE000) >> 13,
//													   (tx_fifo_reg & 0x000003FF)
//						);
			if (tx_fifo_reg != 0xDEADBEEF) {
					debug_printf(LEVEL_INFO, DEBUG_MSK, "DEBUG TX: etv=%x etr=%x ftv=%x ftr=%x txr=%x etl=%x %s wr=%03x rd=%03x\n", 
						(tx_fifo_reg & 0x80000000) >> 31,
						(tx_fifo_reg & 0x40000000) >> 30,
						(tx_fifo_reg & 0x20000000) >> 29,
						(tx_fifo_reg & 0x10000000) >> 28,
						(tx_fifo_reg & 0x08000000) >> 27,
						(tx_fifo_reg & 0x04000000) >> 26,
						tx_state_names[(tx_fifo_reg & 0x03800000) >> 23],
						(tx_fifo_reg & 0x007FE000) >> 13,
						(tx_fifo_reg & 0x000003FF)
						);
				}
			if (rx_fifo_reg != 0xDEADBEEF) {
					debug_printf(LEVEL_INFO, DEBUG_MSK, "DEBUG RX: vstart=%x v.busy=%x v.done=%x dec.tv=%x dec.tr=%x %s wr=%03x rd=%03x\n",
						(rx_fifo_reg & 0x10000000) >> 28,
						(rx_fifo_reg & 0x08000000) >> 27,
						(rx_fifo_reg & 0x04000000) >> 26,
						(rx_fifo_reg & 0x02000000) >> 25,
						(rx_fifo_reg & 0x01000000) >> 24,
						rx_state_names[(rx_fifo_reg & 0xe0000000) >> 29],
						(rx_fifo_reg & 0x007FE000) >> 13,
						(rx_fifo_reg & 0x000003FF)						
						);
				}

			}
			debug_printf(LEVEL_BORING, DEBUG_MSK, "debugthread power %d\n",
					capture_and_read_msk(OFFSET_MSK(rx_power)));
			print_rssi();

			sync_status = capture_and_read_msk(OFFSET_MSK(rx_frame_sync_status));
			frame_sync_locked = sync_status & 0x00000001;
			frame_buffer_overflow = sync_status & 0x00000002;
			frames_received = (sync_status & 0x03fffffc) >> 2;
			frame_sync_errors = ((sync_status & 0xfc000000) >> 26) & 0x3f;
			debug_printf(LEVEL_INFO, DEBUG_MSK, "debugthread frame: raw 0x%08x rcvd %d, errs %d %s %s\n",
				sync_status,
				frames_received,
				frame_sync_errors,
				frame_sync_locked ? "LOCKED" : "unlocked",
				frame_buffer_overflow ? "OVERFLOW" : "");
				
//			debug_printf(LEVEL_INFO, DEBUG_MSK, "debugthread axis: axis_xfer_count = 0x%08x\n",
//					capture_and_read_msk(OFFSET_MSK(axis_xfer_count)));

		debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX RELEASE in debugthread\n");
		pthread_mutex_unlock(&timeline_lock);
		debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX RELEASED in debugthread\n");

//		usleep(10000);	// 10ms nominal sample rate
		usleep(1000);	//!!! try 1ms
	}
	debug_printf(LEVEL_LOW, DEBUG_THREADS, "debug thread exiting\n");
	return NULL;
}

int start_debug_thread(void) {

	if (pthread_create(&ovp_debug_thread, NULL, ovp_debug_thread_func, NULL) != 0) {
		debug_printf(LEVEL_URGENT, DEBUG_THREADS, "OVP: Failed to create debug thread");
		return -1;
	}
	
	debug_printf(LEVEL_INFO, DEBUG_THREADS, "OVP: debug thread started successfully\n");
	return 0;
}

void stop_debug_thread(void) {
	if (ovp_debug_thread) {
		pthread_cancel(ovp_debug_thread);
	}
		
	debug_printf(LEVEL_INFO, DEBUG_THREADS, "OVP: debug thread stopped\n");
}

