
#include <errno.h>
#include <iio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "debug_printf.h"
#include "fec.h"
#include "interleaver.h"
#include "numerology.h"
#include "radio.h"
#include "randomizer.h"
#include "receiver.h"
#include "registers.h"
#include "timestamp.h"
#include "udp_encap.h"

extern bool software_rx_processing;
extern void cleanup_and_exit(int retval);
extern int init_udp_socket(void);
extern struct sockaddr_in udp_client_addr;
//!!!extern struct iio_channel *rx0_i;
extern struct iio_context *ctx;
extern bool stop;

uint64_t ovp_refill_count = 0;
uint64_t ovp_refill_error_count = 0;
uint64_t ovp_forwarded_count = 0;


// Receiver thread
// The receiver thread pulls frames of data from IIO and eventually
// encapsulates them for transmission over the network.
//
// We are assuming that the modem (FPGA) detects the frame sync word
// at the beginning of each frame and uses that to establish frame
// boundaries and byte alignment. The modem consumes the frame sync
// word and does not send it to us.
//
// Having received the frame over the air, the FPGA might or might not
// do more of the processing to extract the decoded data from the frame.
// For now, we are assuming that it does no further processing, and
// simply ships the unprocessed frame to us via IIO.
//
// The receiver thread blocks in a iio_buffer_refill() call with an
// infinite timeout. Initially, before any frame sync word has been
// detected, nothing happens. After a first frame sync word is detected,
// the FPGA gathers up the whole frame and then burst-transfers it to
// IIO. At that point, the iio_buffer_refill() call will return with
// one frame of data (this depends on the size of rxbuf matching the
// size of an unprocessed frame, not including the sync word). While
// further frames are being received, along with their properly-aligned
// frame sync words, each call to iio_buffer_refill() should return
// promptly (at 40ms boundaries) with a new frame full of data.
//
// Eventually the modem will fail to receive frame sync words,
// either because the transmission has ended or because it has faded
// out. The modem may deliver some number of frames full of garbage,
// and is expected to eventually decide that no signal is present and
// stop sending frames to us. When a frame sync is again detected by
// the modem, it may or may not be time-aligned with the previous
// series of frames. Dealing with this is entirely up to the modem
// and to downstream application processors. We just accept frames
// from the modem, encapsulate them for network transmission, and
// forward them along without delay.
//
// libiio is not thread-safe. Recommended solution is to clone the context
// and re-create everything in the cloned context. They have already been
// appropriately configured in main.
//

pthread_t ovp_rx_thread;
struct iio_buffer *rx_buf;
static bool encapsulated_frame_host_known = false;

void* ovp_receiver_thread(__attribute__((unused)) void *arg) {
	uint32_t refill_ts_base;
	ssize_t nbytes_rx;
	uint8_t received_frame[OVP_DEMOD_FRAME_SIZE];
	uint8_t decoded_frame[OVP_SINGLE_FRAME_SIZE];
	
	struct iio_context *rx_ctx;
	struct iio_device *rx_dev;
	struct iio_channel *rx_ch_i, *rx_ch_q;

	// Allegedly, we need to clone the context and recreate all the
	// already-initialized devices and channels, because IIO is not
	// thread-safe.
	rx_ctx = iio_context_clone(ctx);
	if (!rx_ctx) {
		debug_printf(LEVEL_URGENT, DEBUG_IIO, "Failed to create receive context\n");
		cleanup_and_exit(1);
	}

	// set the timeout to infinity; we may need to wait any length of time
	// before the first frame of a transmission is received, so we need the
	// receive buffer_refill to never time out.
	//
	// It appears that this particular setting doesn't carry forward to
	// the cloned context, so it needs to be set anyway.
	int ret = iio_context_set_timeout(rx_ctx, 0);
	if (ret < 0) {
			char timeout_test[256];
			iio_strerror(-(int)ret, timeout_test, sizeof(timeout_test));
			debug_printf(LEVEL_URGENT, DEBUG_IIO, "* rx_ctx set_timeout failed : %s\n", timeout_test);
	} else {
			debug_printf(LEVEL_BORING, DEBUG_IIO, "* rx_ctx set_timeout returned %d, which is a success.\n", ret);
	}


	// Find the SAME devices/channels (already configured by main)
	rx_dev = iio_context_find_device(rx_ctx, "cf-ad9361-lpc");
	rx_ch_i = iio_device_find_channel(rx_dev, "voltage0", RX);
	rx_ch_q = iio_device_find_channel(rx_dev, "voltage1", RX);

	// Enable channels in THIS context
	iio_channel_enable(rx_ch_i);
	iio_channel_enable(rx_ch_q);

	rx_buf = iio_device_create_buffer(rx_dev, software_rx_processing ? OVP_DEMOD_FRAME_SIZE : OVP_SINGLE_FRAME_SIZE, false);
	if (!rx_buf) {
		debug_printf(LEVEL_URGENT, DEBUG_IIO, "Could not create RX buffer");
		cleanup_and_exit(1);
	}


	while(!stop) {
		// Refill RX buffer
		refill_ts_base = get_timestamp_ms();
		nbytes_rx = iio_buffer_refill(rx_buf);
		ovp_refill_count++;
		if (nbytes_rx < 0) {
				if (nbytes_rx == -ETIMEDOUT) {
					debug_printf(LEVEL_INFO, DEBUG_RX, "Refill timeout (no sync word yet?)\n");
					nbytes_rx = 0;
				} else {
					debug_printf(LEVEL_URGENT, DEBUG_IIO, "Error refilling buf %d\n",(int) nbytes_rx);
					ovp_refill_error_count++;
					cleanup_and_exit(1);
				}
		} else {
			debug_printf(LEVEL_INFO, DEBUG_IIO, "buffer_refill of %d bytes took %dms\n", nbytes_rx / 4, get_timestamp_ms() - refill_ts_base);

			// nbytes_rx includes the three wasted bytes for each byte transferred via AXI-S
			if (nbytes_rx != (software_rx_processing ? OVP_DEMOD_FRAME_SIZE : OVP_SINGLE_FRAME_SIZE) * 4) {
				debug_printf(LEVEL_MEDIUM, DEBUG_IIO, "Warning: unexpected rx frame size %d; discarded!\n", nbytes_rx);
				ovp_refill_error_count++;
				continue;
			}

			// make a copy of the received data, removing IIO padding
			ptrdiff_t p_inc = iio_buffer_step(rx_buf);
			char *p_end = iio_buffer_end(rx_buf);
			uint8_t *p_out = received_frame;
			char *first = (char *)iio_buffer_first(rx_buf, rx_ch_i);
			for (char *p_dat = first; p_dat < p_end; p_dat += p_inc) {
				*p_out++ = ((int16_t*)p_dat)[0] & 0x00ff;
			}

			// dump received buffer for visual check
			printf("Received raw data @ %p: ", first);
			for (int i=0; i < (software_rx_processing? OVP_DEMOD_FRAME_SIZE : OVP_SINGLE_FRAME_SIZE); i++) {
				printf("%02x ", received_frame[i]);
			}
			printf("\n");

			if (software_rx_processing) {
				// remove the interleaving so the decoder can do its work
				deinterleave_ovp_frame(received_frame);

				// decode the whole frame
				decode_ovp_frame(received_frame, decoded_frame);

				// remove the randomization
				randomize_ovp_frame(decoded_frame);
			} else {
				memcpy(decoded_frame, received_frame, OVP_SINGLE_FRAME_SIZE);
			}

			//!!! dump logical data for visual check
			printf("Received processed data: ");
			for (int i=0; i < OVP_SINGLE_FRAME_SIZE; i++) {
				printf("%02x ", decoded_frame[i]);
			}
			printf("\n");

			if (!encapsulated_frame_host_known) {
				printf("OTA frame received before first UDP datagram; discarded.");
				continue;
			}

			forward_encap_frame(decoded_frame, OVP_SINGLE_FRAME_SIZE);
			ovp_forwarded_count++;
		}
	}

	iio_buffer_destroy(rx_buf);
	iio_context_destroy(rx_ctx);

	debug_printf(LEVEL_BORING, DEBUG_THREADS, "receiver thread exiting\n");
	return NULL;
}

// Initialize OVP receiver thread
int init_ovp_receiver(void) {

	if (init_udp_socket()) {
		return -1;
	}

	encapsulated_frame_host_known = false;
	return 0;
}


// Start OVP receiver thread
int start_ovp_receiver(void) {
	if (init_ovp_receiver() < 0) {
		return -1;
	}
		
	if (pthread_create(&ovp_rx_thread, NULL, ovp_receiver_thread, NULL) != 0) {
		debug_printf(LEVEL_URGENT, DEBUG_THREADS, "Failed to create receiver thread");
		return -1;
	}
	
	debug_printf(LEVEL_BORING, DEBUG_RX, "Receiver started successfully\n");
	return 0;
}

// Stop OVP receiver thread
void stop_ovp_receiver(void) {
	if (ovp_rx_thread) {
		if (rx_buf) {
			iio_buffer_cancel(rx_buf);
		}
		pthread_cancel(ovp_rx_thread);
	}
		
	debug_printf(LEVEL_BORING, DEBUG_RX, "receiver stopped\n");
}

void receiver_ok_to_forward_frames(bool ok) {
    encapsulated_frame_host_known = ok;

	if (ok) {
		register_encap_address(&udp_client_addr);
	}
}
