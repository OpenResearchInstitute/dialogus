
#include <iio.h>
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "debug_printf.h"
#include "numerology.h"
#include "registers.h"
#include "statistics.h"
#include "timestamp.h"
#include "tx_timeline.h"


extern bool software_tx_processing;
extern bool stop;
extern volatile int ovp_transmission_active;
extern pthread_mutex_t tls_lock;
extern pthread_mutex_t timeline_lock;
extern pthread_cond_t timeline_start;
extern int64_t session_T0;
extern bool hang_timer_active;
extern int hang_timer_frames;
extern uint8_t logical_frame_buffer[];
extern uint8_t modulator_frame_buffer[];
extern struct iio_buffer *txbuf;
extern void create_postamble_logical_frame(void);
extern void create_dummy_logical_frame(void);
void process_ovp_frame_in_software(uint8_t *ovp_frame);
extern int load_ovp_frame_into_txbuf(uint8_t *frame_data, size_t frame_size);
extern void end_transmission_session_normally(void);


static pthread_t ovp_timeline_thread;

static int64_t decision_time = 0;	// us timestamp after which a new frame is late
static int frames_readied_for_push = 0;	// number of UDP frames seen before decision time (should be 1)


// Call this function to set a new decision time.
void tx_timeline_set_decision_time(int64_t new_decision_time) {
	decision_time = new_decision_time;
}

// Call this function each time a transmit frame has been fully
// prepared (in logical_frame_buffer, and also in modulator_frame_buffer
// if software_tx_processing is enabled).
// If it has not been called before the decision time, we don't have a
// frame to transmit on time and must vamp with a dummy frame.
// If it has been called more than once before the decision time,
// all but the last one will have been overwritten; the lost
// frames are designated "untimely".
void tx_timeline_frame_ready(void) {
	frames_readied_for_push++;
}

// OVP Timeline Manager thread
// Wakes up at each frame decision time (Td) during an active session
// and sends a frame. In priority order:
//		* A real frame if one is available, or
//		* A postamble frame if the final hang time is completed, or
//		* A dummy frame.
void* ovp_timeline_manager_thread(__attribute__((unused)) void *arg) {
	int64_t now;
	static int hang_time_dummy_count = 0;
	uint32_t local_ts_base;

	while (!stop) {
		while (ovp_transmission_active) {
			// printf("timeline grabbing mutex\n");
			debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX timeline_lock: locking in timeline\n");
			pthread_mutex_lock(&timeline_lock);
			debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX timeline_lock: acquired in timeline\n");

			if (decision_time != 0) {
				now = get_timestamp_us();
				debug_printf(LEVEL_INFO, DEBUG_TIMELINE, "now %lld Td %lld -> decision in %lld us\n", now, decision_time, decision_time - now);
				if (now >= decision_time) {
					if (frames_readied_for_push > 0) {
						if (frames_readied_for_push > 1) {
							ovp_untimely_frames += frames_readied_for_push - 1;
							debug_printf(LEVEL_INFO, DEBUG_TIMELINE, "timeline @ %lld: frame + %d untimely frames\n", get_timestamp_us() - session_T0, frames_readied_for_push - 1);
						} else {
							debug_printf(LEVEL_INFO, DEBUG_TIMELINE, "timeline @ %lld: frame\n", get_timestamp_us() - session_T0);
						}
						hang_time_dummy_count = 0;
						hang_timer_active = false;
					} else {
						if (hang_time_dummy_count >= hang_timer_frames) {
							create_postamble_logical_frame();
							debug_printf(LEVEL_INFO, DEBUG_TIMELINE, "timeline @ %lld: postamble\n", get_timestamp_us() - session_T0);
							hang_time_dummy_count = 0;
							hang_timer_active = false;
							ovp_transmission_active = 0;
						} else {
							create_dummy_logical_frame();
							debug_printf(LEVEL_INFO, DEBUG_TIMELINE, "timeline @ %lld: dummy frame\n", get_timestamp_us() - session_T0);
							ovp_dummy_frames_sent++;
							hang_time_dummy_count++;
							hang_timer_active = true;
						}
					}

					// Now on a common path for all outgoing frames: real, dummy, or postamble
					if (software_tx_processing) {
						debug_printf(LEVEL_BORING, DEBUG_TIMELINE, "Attempting software processing\n");
						process_ovp_frame_in_software(logical_frame_buffer);
						load_ovp_frame_into_txbuf(modulator_frame_buffer, OVP_MODULATOR_FRAME_SIZE);
					} else {
						debug_printf(LEVEL_BORING, DEBUG_TIMELINE, "Letting the hardware handle processing\n");
						load_ovp_frame_into_txbuf(logical_frame_buffer, OVP_SINGLE_FRAME_SIZE);
					}
					local_ts_base = get_timestamp_ms();
					ssize_t result = iio_buffer_push(txbuf);
					debug_printf(LEVEL_INFO, DEBUG_TIMELINE, "iio_buffer_push took %dms.\n", get_timestamp_ms()-local_ts_base);
					if (result < 0) {
						debug_printf(LEVEL_URGENT, DEBUG_TIMELINE, "Error pushing buffer to MSK: %zd\n", result);
					}
					debug_printf(LEVEL_INFO, DEBUG_TIMELINE, "After push, at %d, axis_xfer_count = 0x%08x tx_bit_count = 0x%08x\n",
							get_timestamp_ms(),
							capture_and_read_msk(OFFSET_MSK(axis_xfer_count)),
							capture_and_read_msk(OFFSET_MSK(Tx_Bit_Count)));

					decision_time += 40e3;
					frames_readied_for_push = 0;
				}

				// handle normal end of a transmission session (postamble was just pushed)
				if (! ovp_transmission_active) {
					end_transmission_session_normally();
				}
			}

			debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX RELEASE in timeline\n");
			pthread_mutex_unlock(&timeline_lock);
			debug_printf(LEVEL_INFO, DEBUG_MUTEX, "MUTEX RELEASED in timeline\n");

			now = get_timestamp_us();
			debug_printf(LEVEL_BORING, DEBUG_TIMELINE, "Getting ready to wait for decision time\n");
			if (decision_time - now > 0) {
				usleep(decision_time - now);	// wait until next decision time
			}
		}

		debug_printf(LEVEL_INFO, DEBUG_TIMELINE, "Timeline paused until next transmission\n");
		pthread_cond_wait(&timeline_start, &tls_lock);
	}

	debug_printf(LEVEL_BORING, DEBUG_TIMELINE, "Timeline manager thread exiting\n");
	return NULL;
}

int start_timeline_manager(void) {
	if (pthread_create(&ovp_timeline_thread, NULL, ovp_timeline_manager_thread, NULL) != 0) {
		perror("Failed to create timeline manager thread");
		return -1;
	}

	debug_printf(LEVEL_BORING, DEBUG_TIMELINE, "Timeline manager started successfully\n");
	return 0;
}

void stop_timeline_manager(void) {
	if (ovp_timeline_thread) {
		pthread_cancel(ovp_timeline_thread);
	}

	debug_printf(LEVEL_BORING, DEBUG_TIMELINE, "Timeline manager stopped\n");
}

